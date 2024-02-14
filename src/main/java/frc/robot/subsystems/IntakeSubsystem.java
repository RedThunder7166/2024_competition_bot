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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_extenderMotor = new TalonFX(IntakeConstants.EXTENDER_MOTOR_ID);
  private final CANcoder m_extenderCancoder = new CANcoder(IntakeConstants.EXTENDER_CANCODER_ID);
  private final TalonFX m_suckerMotor = new TalonFX(IntakeConstants.SUCKER_MOTOR_ID);

  // private final DutyCycleOut m_intakeRequest = new DutyCycleOut(0);
  private final MotionMagicDutyCycle m_extenderExtendedRequest = new MotionMagicDutyCycle(
    Utils.angleToEncoderUnits(IntakeConstants.EXTENDER_CANCODER_EXTENDED_POSITION)
  );
  private final MotionMagicDutyCycle m_extenderHomeRequest = new MotionMagicDutyCycle(
    Utils.angleToEncoderUnits(IntakeConstants.EXTENDER_CANCODER_HOME_POSITION)
  );

  private final VelocityDutyCycle m_suckerRequest = new VelocityDutyCycle(IntakeConstants.SUCKER_TARGET_VELOCITY_RPS);

  private boolean m_enabledState = false;
  
  public IntakeSubsystem() {
    final TalonFXConfiguration extender_motor_configs = new TalonFXConfiguration();

    // TODO: tune this
    final Slot0Configs extender_slot0_configs = extender_motor_configs.Slot0;
    extender_slot0_configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    extender_slot0_configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    extender_slot0_configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    extender_slot0_configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    extender_slot0_configs.kI = 0; // no output for integrated error
    extender_slot0_configs.kD = 0; // no output for error derivative

    // TODO: tune this
    final MotionMagicConfigs extender_motion_magic_configs = extender_motor_configs.MotionMagic;
    extender_motion_magic_configs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    extender_motion_magic_configs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_extenderMotor.setNeutralMode(NeutralModeValue.Brake);
    m_extenderMotor.getConfigurator().apply(extender_motor_configs, 0.05);

    m_suckerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    if (m_enabledState) {
      m_extenderMotor.setControl(m_extenderExtendedRequest);
      m_suckerMotor.setControl(m_suckerRequest);
    } else {
      m_extenderMotor.setControl(m_extenderHomeRequest);
      m_suckerMotor.disable();
    }
  }

  // public void startIntake() {
  //   m_intakeMotor.setControl(m_intakeRequest.withOutput(1));
  // }

  // public void stopIntake() {
  //   m_intakeMotor.disable();
  // }

  public InstantCommand m_deployAndStartCommand = new InstantCommand(() -> {
    m_enabledState = true;
  }, this);
  public InstantCommand m_retractAndStopCommand = new InstantCommand(() -> {
    m_enabledState = false;
  }, this);
}
