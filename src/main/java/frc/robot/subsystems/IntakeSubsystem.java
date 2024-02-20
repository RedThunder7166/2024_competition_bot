// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_suckerMotor = new TalonFX(IntakeConstants.SUCKER_MOTOR_ID);

  private final VelocityDutyCycle m_suckerRequest = new VelocityDutyCycle(IntakeConstants.SUCKER_TARGET_VELOCITY_RPS);

  private boolean m_forwardState = false;
  private boolean m_reverseState = false;
  
  public IntakeSubsystem() {
    m_suckerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // TODO: PUT THIS BACK (REMOVED FOR MANUAL TESTING)
    // if (m_enabledState) {
    //   m_actuatorMotor.setControl(m_actuatorExtendedRequest);
    //   m_suckerMotor.setControl(m_suckerRequest);
    // } else {
    //   m_actuatorMotor.setControl(m_actuatorHomeRequest);
    //   m_suckerMotor.disable();
    // }

    if (m_forwardState) {
      m_suckerMotor.setControl(m_suckerRequest.withVelocity(IntakeConstants.SUCKER_TARGET_VELOCITY_RPS));
    } else if (m_reverseState) {
      m_suckerMotor.setControl(m_suckerRequest.withVelocity(-IntakeConstants.SUCKER_TARGET_VELOCITY_RPS));
    } else {
      m_suckerMotor.disable();
    }
  }

  public void manualSuckDeleteMe(double value) {
    m_suckerMotor.setControl(new DutyCycleOut(value));
  }

  public void manualSuckStopDeleteMe() {
    m_suckerMotor.disable();
  }

  public final InstantCommand m_startForwardCommand = new InstantCommand(() -> {
    m_forwardState = true;
  }, this);
  public final InstantCommand m_stopForwardCommand = new InstantCommand(() -> {
    m_forwardState = false;
  }, this);

  public final InstantCommand m_startReverseCommand = new InstantCommand(() -> {
    m_reverseState = true;
  }, this);
  public final InstantCommand m_stopReverseCommand = new InstantCommand(() -> {
    m_reverseState = false;
  }, this);
}
