// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.AimLocation;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_upperMotor = new TalonFX(IntakeConstants.UPPER_MOTOR_ID);
  private final TalonFX m_lowerMotor = new TalonFX(IntakeConstants.LOWER_MOTOR_ID);

  private final DigitalInput m_entranceSensor = new DigitalInput(IntakeConstants.ENTRANCE_SENSOR_ID);
  // private final DigitalInput m_exitSensor = new DigitalInput(IntakeConstants.EXIT_SENSOR_ID);

  private boolean m_forwardEnabled = false;
  private boolean m_reverseEnabled = false;

  // private boolean m_entranceSensorIsTripped = false;
  private boolean m_exitSensorIsTripped = false;
  private boolean m_entranceSensorIsTripped = false;
  private final ShuffleboardTab m_sensorTab = Shuffleboard.getTab("Sensors");

  private final IndexerSubsystem m_indexer;

  public IntakeSubsystem(IndexerSubsystem indexer) {
    m_indexer = indexer;
    // m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // clockwise is true
    // counterclockwise is false
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    
    m_upperMotor.getConfigurator().apply(config);
    m_lowerMotor.getConfigurator().apply(config);
    
    m_lowerMotor.setControl(new Follower(m_upperMotor.getDeviceID(), false));

    m_sensorTab.addBoolean("IntakeEntranceSensor", () -> m_entranceSensorIsTripped);
  }

  private static final double speed = 0.45;

  private final DutyCycleOut m_forwardControl = new DutyCycleOut(speed);
  private final DutyCycleOut m_reverseControl = new DutyCycleOut(-speed);
  @Override
  public void periodic() {
    m_entranceSensorIsTripped = Utils.isAllenBradleyTripped(m_entranceSensor);
    // m_exitSensorIsTripped = Utils.isAllenBradleyTripped(m_exitSensor);

    if (m_forwardEnabled) {
      // ANY CHANGES BELOW NEED TO BE REFLECTED IN THE SUBSEQUENT INDEXER LOGIC
      if (AimLocation.getAimLocation() != AimLocation.Loading && m_indexer.getSensorTripped()) {
        m_upperMotor.disable();
      } else {
        m_upperMotor.setControl(m_forwardControl);
      }
    } else if (m_reverseEnabled) {
      m_upperMotor.setControl(m_reverseControl);
    } else {
      m_upperMotor.disable();
    }
  }

  public boolean getEntranceSensorTripped() {
    return m_entranceSensorIsTripped;
  }
  public boolean getExitSensorTripped() {
    return m_exitSensorIsTripped;
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