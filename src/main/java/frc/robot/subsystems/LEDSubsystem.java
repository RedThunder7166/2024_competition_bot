// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

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

  private static final RGB yellow = new RGB(248, 229, 89);
  private static final RGB green = new RGB(154, 222, 123);
  private static final RGB red = new RGB(205, 4, 4);

  private static final double brightness = 0.5;
  private static final double speed = 0.5;
  private static enum Mode {
    Default(yellow),
    Rainbow(new RainbowAnimation(brightness, speed, LEDConstants.LED_COUNT)),
    FlashYellow(singleFadeAnimation(yellow, speed)),
    FlashGreen(singleFadeAnimation(green, speed)),
    FlashRed(singleFadeAnimation(red, speed)),
    SolidYellow(yellow),
    SolidGreen(green),
    SolidRed(red);

    public final boolean has_animation;

    public final int r;
    public final int g;
    public final int b;

    public final Animation animation;

    Mode(int r, int g, int b) {
      this.has_animation = false;
      this.r = r;
      this.g = g;
      this.b = b;
      this.animation = null;
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
  }
  private Mode m_mode = Mode.Default;

  private final CANdle m_candle = new CANdle(LEDConstants.CANDLE_ID);

  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public LEDSubsystem(IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_intake = intake;
    m_shooter = shooter;

    final CANdleConfiguration config = new CANdleConfiguration();
    config.disableWhenLOS = true;
    config.stripType = LEDStripType.RGB;

    m_candle.configAllSettings(config, 100);
  }

  @Override
  public void periodic() {
    if (m_intake.getEntranceSensorTripped()) {
      m_mode = Mode.FlashGreen;
    } else if (m_intake.getExitSensorTripped()) {
      m_mode = Mode.SolidGreen;
    } else if (m_shooter.getWheelExitSensorTripped()) { // redundant as default is equal to default yellow
      m_mode = Mode.SolidYellow;
    } else {
      m_mode = Mode.Default;
    }

    if (m_mode.has_animation) {
      m_candle.animate(m_mode.animation);
    } else {
      m_candle.setLEDs(m_mode.r, m_mode.g, m_mode.b,
      0, LEDConstants.START_INDEX, LEDConstants.LED_COUNT
      );
    }
  }
}
