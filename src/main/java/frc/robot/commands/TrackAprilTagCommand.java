// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class TrackAprilTagCommand extends Command {
  /** Creates a new AimAtTarget. */
  private final CommandSwerveDrivetrain m_driveTrain;
  private final VisionSubsystem m_visionSubsystem;
  private final SwerveRequest.FieldCentric m_driveRequest;
  private final double MaxAngularRate;
  public TrackAprilTagCommand(CommandSwerveDrivetrain drive, VisionSubsystem vision, SwerveRequest.FieldCentric drive_request, double max_angular_rate) {
    addRequirements(drive, vision);

    m_driveTrain = drive;
    m_visionSubsystem = vision;
    m_driveRequest = drive_request;
    MaxAngularRate = max_angular_rate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn_power = m_visionSubsystem.calculateTurnPower();
    m_driveTrain.setControl(m_driveRequest.withRotationalRate(turn_power * MaxAngularRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
