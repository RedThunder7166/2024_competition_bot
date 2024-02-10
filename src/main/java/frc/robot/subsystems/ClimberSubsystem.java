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
  private final TalonFX leftClimbMotor = new TalonFX(ClimberConstants.LEFT_CLIMB_MOTOR_ID);
  private final TalonFX rightClimbMotor = new TalonFX(ClimberConstants.RIGHT_CLIMB_MOTOR_ID);
  private final DigitalInput rightArmInput = new DigitalInput(ClimberConstants.RIGHT_ARM_LIMIT_SWITCH_ID);
  private final DigitalInput leftArmInput = new DigitalInput(ClimberConstants.LEFT_ARM_LIMIT_SWITCH_ID);

  private final CommandSwerveDrivetrain m_swerve;

  private double m_percent = 0;
    
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
  }

  public double getRollDegrees() {
    return Math.toDegrees(m_swerve.getRotation3d().getX());
  }

  public void driveArms(double left_percent, double right_percent) {
    boolean left_switch_is_down = !leftArmInput.get();
    if (left_switch_is_down && left_percent <= 0) {
      leftClimbMotor.set(0);
    } else {
      leftClimbMotor.setControl(new DutyCycleOut(left_percent));
    }

    boolean right_switch_is_down = !rightArmInput.get();
    if (right_switch_is_down && right_percent <= 0) {
      rightClimbMotor.set(0);
    } else {
      rightClimbMotor.setControl(new DutyCycleOut(right_percent));
    }
  }

  @Override
  public void periodic() {
    double roll = getRollDegrees();

    double left_percent = m_percent;
    double right_percent = m_percent;
    if (roll >= 5) {
      // left_percent = (left_percent < 0) ? (left_percent + 0.5) : (left_percent - 0.5);
      left_percent /= 2;
    } else if (roll <= -5) {
      right_percent /= 2;
    }

    driveArms(left_percent, right_percent);
  }

  public void setPercent(double percent) {
    m_percent = percent;
  }
}