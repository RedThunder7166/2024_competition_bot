// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_upperMotor = new TalonFX(IntakeConstants.UPPER_MOTOR_ID);
  private final TalonFX m_lowerMotor = new TalonFX(IntakeConstants.LOWER_MOTOR_ID);

  // private final VelocityDutyCycle m_request = new VelocityDutyCycle(IntakeConstants.TARGET_VELOCITY_RPS);

  private boolean m_forwardState = false;
  private boolean m_reverseState = false;
  
  public IntakeSubsystem() {
    // m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration config = new TalonFXConfiguration();
    // clockwise is true
    // counterclockwise is false
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    m_upperMotor.getConfigurator().apply(config);
    m_lowerMotor.getConfigurator().apply(config);

    m_lowerMotor.setControl(new Follower(m_upperMotor.getDeviceID(), false));
  }

  private final DutyCycleOut m_forwardControl = new DutyCycleOut(0.2);
  private final DutyCycleOut m_reverseControl = new DutyCycleOut(-0.2);
  @Override
  public void periodic() {
    // System.out.format("Forward: %s, Reverse: %s\n", m_forwardState, m_reverseState);
    if (m_forwardState) {
      // m_upperMotor.setControl(m_request.withVelocity(IntakeConstants.TARGET_VELOCITY_RPS));
      m_upperMotor.setControl(m_forwardControl);
    } else if (m_reverseState) {
      // m_upperMotor.setControl(m_request.withVelocity(-IntakeConstants.TARGET_VELOCITY_RPS));
      m_upperMotor.setControl(m_reverseControl);
    } else {
      m_upperMotor.disable();
    }
  }

 

  public void disabledInit() {
    m_forwardState = false;
    m_reverseState = false;
  }

 

  public void enableForward() {
    m_forwardState = true;
  }
  public void disableForward() {
    m_forwardState = false;
  }

  public void enableReverse() {
    m_reverseState = true;
  }
  public void disableReverse() {
    m_reverseState = false;
  }
}
