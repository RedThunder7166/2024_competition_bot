// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.IndexerConstants;



public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(IndexerConstants.MOTOR_ID);
  private static final double speed = 0.65;
  private final DutyCycleOut m_forwardControl = new DutyCycleOut(speed);
  private final DutyCycleOut m_reverseControl = new DutyCycleOut(-speed);

  // private final DigitalInput m_entranceSensor = new DigitalInput(IndexerConstants.ENTRANCE_SENSOR_ID);

  private boolean m_forwardEnabled = false;
  private boolean m_reverseEnabled = false;

  // private boolean m_entranceSensorIsTripped = false;
  private boolean m_sensorIsTripped = false;

  private final ShooterSubsystem m_shooter;

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("IndexerMotor");

  private final ShuffleboardTab m_sensorTab = Shuffleboard.getTab("Sensors");

  public IndexerSubsystem(ShooterSubsystem shooter) {
    m_shooter = shooter;

    m_shuffleBoardTab.addDouble("MotorTemp", () -> m_motor.getDeviceTemp().getValueAsDouble());

    TalonFXConfiguration config = new TalonFXConfiguration();
    // clockwise is true
    // counterclockwise is false
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    m_motor.getConfigurator().apply(config);

    m_sensorTab.addBoolean("IndexerSensor", () -> m_sensorIsTripped);
  }

  @Override
  public void periodic() {
    // m_entranceSensorIsTripped = Utils.isAllenBradleyTripped(m_entranceSensor);

    if (m_forwardEnabled) {
      m_motor.setControl(m_forwardControl);
    } else if (m_reverseEnabled) {
      m_motor.setControl(m_reverseControl);
    } else {
      m_motor.disable();
    }
  }

  public boolean getSensorTripped() {
    return m_sensorIsTripped;
  }

  public void disabledInit() {
    m_forwardEnabled = false;
    m_reverseEnabled = false;
  }

  public void enableForward() {
    m_forwardEnabled = true;
  }
  public void disableForward() {
    m_forwardEnabled = false;
  }

  public void enableReverse() {
    m_reverseEnabled = true;
  }
  public void disableReverse() {
    m_reverseEnabled = false;
  }
}