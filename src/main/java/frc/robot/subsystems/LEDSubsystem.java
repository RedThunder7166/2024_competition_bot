// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {  
  private static final class RGB {
    public final int r;
    public final int g;
    public final int b;
    private RGB(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  private static SingleFadeAnimation singleFadeAnimation(RGB rgb, double speed) {
    return new SingleFadeAnimation(rgb.r, rgb.g, rgb.b, 0, speed, LEDConstants.LED_COUNT, LEDConstants.START_INDEX);
  }
  private static StrobeAnimation strobeAnimation(RGB rgb, double speed) {
    return new StrobeAnimation(rgb.r , rgb.g, rgb.b, 0, speed, LEDConstants.LED_COUNT, LEDConstants.START_INDEX);
  }
  private static ColorFlowAnimation colorFlowAnimation(RGB rgb, double speed, ColorFlowAnimation.Direction direction) {
    return new ColorFlowAnimation(rgb.r, rgb.g, rgb.b, 0, speed, LEDConstants.LED_COUNT, direction, LEDConstants.START_INDEX);
  }

  private static final RGB green = new RGB(0, 255, 0);
  private static final RGB red = new RGB(230, 20, 8);
  private static final RGB blue = new RGB(40, 40, 250);
  private static final RGB yellow = new RGB(242, 187, 5);
  private static final RGB orange = new RGB(250, 90, 0);

  private static final double brightness = 1;
  private static final double speed = 0.5;
  private static enum Mode {
    Rainbow(new RainbowAnimation(brightness, speed, LEDConstants.LED_COUNT)),
    FlashYellow(strobeAnimation(yellow, speed)),
    FlashGreen(strobeAnimation(green, speed)),
    FlashRed(strobeAnimation(red, speed)),
    SolidYellow(yellow),
    SolidGreen(green),
    SolidBlue(blue),
    SolidRed(red),
    SolidOrange(orange),
    BlueFlow(colorFlowAnimation(blue, speed, ColorFlowAnimation.Direction.Forward)),
    FlashOrange(strobeAnimation(orange, speed));

    public final boolean has_animation;

    public final int r;
    public final int g;
    public final int b;

    public final Animation animation;

    private int offset;

    Mode(int r, int g, int b) {
      this.has_animation = false;
      this.r = r;
      this.g = g;
      this.b = b;
      this.animation = null;
      this.offset = 0;
    }
    Mode(RGB rgb) {
      this(rgb.r, rgb.g, rgb.b);
    }
    Mode(Animation animation) {
      this.has_animation = true;
      this.r = 0;
      this.g = 0;
      this.b = 0;
      this.animation = animation;
    }

    private void updateAnimationOffset() {
      if (has_animation) {
        animation.setLedOffset(offset);
      }
    }

    public Mode group1() {
      offset = LEDConstants.GROUP_1_START;
      updateAnimationOffset();

      return this;
    }
    public Mode group2() {
      offset = LEDConstants.GROUP_2_START;
      updateAnimationOffset();

      return this;
    }
    public Mode group3() {
      offset = LEDConstants.GROUP_3_START;
      updateAnimationOffset();

      return this;
    }
  }
  // private Mode m_mode = Mode.SolidYellow;
  private Mode m_outerMode = Mode.SolidYellow;
  private Mode m_innerMode = Mode.SolidYellow;

  private final CANdle m_candle = new CANdle(LEDConstants.CANDLE_ID);

  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;
  private final VisionSubsystemLimelight m_vision;

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("LED");

  public LEDSubsystem(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, VisionSubsystemLimelight vision) {
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;
    m_vision = vision;

    final CANdleConfiguration config = new CANdleConfiguration();
    config.disableWhenLOS = false; //Changed from False
    config.stripType = LEDStripType.RGB;

    m_candle.configAllSettings(config, 100);

    // m_shuffleBoardTab.addString("mode", () -> m_mode.name());
    m_shuffleBoardTab.addString("inner mode", () -> m_innerMode.name());
    m_shuffleBoardTab.addString("outer mode", () -> m_outerMode.name());
  }

  @Override
  public void periodic() {
    if (m_shooter.isUpToSpeed()) {
      m_outerMode = Mode.FlashOrange;
    } else if (m_intake.getEntranceSensorTripped()) {
      m_outerMode = Mode.FlashGreen;
    } else if (m_shooter.getWheelExitSensorTripped()) { 
      m_outerMode = Mode.SolidRed;
    } else if (m_indexer.getSensorTripped()) {
      m_outerMode = Mode.FlashYellow;
    } else {
      m_outerMode = Mode.SolidYellow;
    }

    if (m_vision.seesAprilTag()) {
      // m_innerMode = Mode.BlueFlow;
      m_innerMode = Mode.SolidBlue;
    } else {
      m_innerMode = Mode.SolidRed;
    }

    if (m_outerMode.has_animation) {
      m_candle.animate(m_outerMode.group1().animation, 0);
      m_candle.animate(m_outerMode.group3().animation, 2);
    } else {
      m_candle.animate(null, 0);
      m_candle.animate(null, 2);

      m_candle.setLEDs(m_outerMode.r, m_outerMode.g, m_outerMode.b,
        0, LEDConstants.GROUP_1_START, LEDConstants.COUNT_PER_GROUP
      );
      m_candle.setLEDs(m_outerMode.r, m_outerMode.g, m_outerMode.b,
        0, LEDConstants.GROUP_3_START, LEDConstants.COUNT_PER_GROUP
      );
    }

    if (m_innerMode.has_animation) {
      m_candle.animate(m_innerMode.group2().animation, 1);
    } else {
      m_candle.animate(null, 1);

      m_candle.setLEDs(m_innerMode.r, m_innerMode.g, m_innerMode.b,
        0, LEDConstants.GROUP_2_START, LEDConstants.COUNT_PER_GROUP
      );
    }
  }
}
