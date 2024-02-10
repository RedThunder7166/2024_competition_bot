// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_extenderMotor = new TalonFX(IntakeConstants.EXTENDER_MOTOR_ID);
  private final CANcoder m_extenderCancoder = new CANcoder(IntakeConstants.EXTENDER_CANCODER_ID);
  private final TalonFX m_intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    
  }
}
