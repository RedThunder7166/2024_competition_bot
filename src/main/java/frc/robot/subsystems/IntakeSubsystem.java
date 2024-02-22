// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_upperMotor = new TalonFX(IntakeConstants.UPPER_MOTOR_ID);
  private final TalonFX m_lowerMotor = new TalonFX(IntakeConstants.LOWER_MOTOR_ID);

  private final VelocityDutyCycle m_request = new VelocityDutyCycle(IntakeConstants.TARGET_VELOCITY_RPS);

  private boolean m_forwardState = false;
  private boolean m_reverseState = false;
  
  public IntakeSubsystem() {
    // m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    // clockwise is true
    // counterclockwise is false
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_upperMotor.getConfigurator().apply(configs);
    m_lowerMotor.getConfigurator().apply(configs);

    m_lowerMotor.setControl(new Follower(m_upperMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    double speed = 0.2;
    // System.out.format("Forward: %s, Reverse: %s\n", m_forwardState, m_reverseState);
    if (m_forwardState) {
      // m_upperMotor.setControl(m_request.withVelocity(IntakeConstants.TARGET_VELOCITY_RPS));
      m_upperMotor.setControl(new DutyCycleOut(speed));
    } else if (m_reverseState) {
      // m_upperMotor.setControl(m_request.withVelocity(-IntakeConstants.TARGET_VELOCITY_RPS));
      m_upperMotor.setControl(new DutyCycleOut(-speed));
    } else {
      m_upperMotor.disable();
    }
  }

  // public void manualSuckDeleteMe(double value) {
  //   m_suckerMotor.setControl(new DutyCycleOut(value));
  // }

  // public void manualSuckStopDeleteMe() {
  //   m_suckerMotor.disable();
  // }

  public void disabledInit() {
    m_forwardState = false;
    m_reverseState = false;
  }

  public final InstantCommand m_enableForwardCommand = new InstantCommand(() -> {
    m_forwardState = true;
  }, this);
  public final InstantCommand m_disableForwardCommand = new InstantCommand(() -> {
    m_forwardState = false;
  }, this);

  public final InstantCommand m_enableReverseCommand = new InstantCommand(() -> {
    m_reverseState = true;
  }, this);
  public final InstantCommand m_disableReverseCommand = new InstantCommand(() -> {
    m_reverseState = false;
  }, this);
}
