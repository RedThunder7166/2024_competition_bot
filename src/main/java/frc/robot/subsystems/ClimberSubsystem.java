// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_leftClimbMotor = new TalonFX(ClimberConstants.LEFT_CLIMB_MOTOR_ID);
  private final TalonFX m_rightClimbMotor = new TalonFX(ClimberConstants.RIGHT_CLIMB_MOTOR_ID);
  private final DigitalInput m_rightArmInput = new DigitalInput(ClimberConstants.RIGHT_ARM_LIMIT_SWITCH_ID);
  private final DigitalInput m_leftArmInput = new DigitalInput(ClimberConstants.LEFT_ARM_LIMIT_SWITCH_ID);

  private final CommandSwerveDrivetrain m_swerve;

  private double m_output = 0;
  private final DutyCycleOut m_leftRequest = new DutyCycleOut(m_output);
  private final DutyCycleOut m_rightRequest = new DutyCycleOut(m_output);

  public ClimberSubsystem(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
  }

  public double getRollDegrees() {
    return Math.toDegrees(m_swerve.getRotation3d().getX());
  }

  public void driveArms(double left_output, double right_output) {
    boolean left_switch_is_down = !m_leftArmInput.get();
    if (left_switch_is_down && left_output <= 0) {
      m_leftClimbMotor.set(0);
    } else {
      m_leftClimbMotor.setControl(m_leftRequest.withOutput(left_output));
    }

    boolean right_switch_is_down = !m_rightArmInput.get();
    if (right_switch_is_down && right_output <= 0) {
      m_rightClimbMotor.set(0);
    } else {
      m_rightClimbMotor.setControl(m_rightRequest.withOutput(right_output));
    }
  }

  @Override
  public void periodic() {
    double roll = getRollDegrees();

    double left_output = m_output;
    double right_output = m_output;
    if (roll >= 5) {
      // left_percent = (left_percent < 0) ? (left_percent + 0.5) : (left_percent - 0.5);
      left_output /= 2;
    } else if (roll <= -5) {
      right_output /= 2;
    }

    driveArms(left_output, right_output);
  }

  // -1 to 1
  public void setOutput(double output) {
    m_output = output;
  }
}