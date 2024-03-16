// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveFieldCentricCommand extends Command {
  private final static double DISTANCE_DEADBAND = 0.1; // TODO: tune this. this should be the desired distance between your target position and your current

  private final CommandSwerveDrivetrain m_swerve;
  private final SwerveRequest.FieldCentric m_request;
  private final double m_distanceMeters;
  private final Translation2d m_initialTranslation;
  private final double m_initialRotationRadians;
  private final double m_targetRotationRadians;

  public static final double TURN_kA = 0;
  public static final double TURN_kS = 0.05;
  public static final double TURN_kV = 0.1;

  public DriveFieldCentricCommand(CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric request,
    double distance_meters, double direction_radians, double rotation_radians
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  
    m_swerve = swerve;
    m_request = request;
    
    m_distanceMeters = distance_meters;
    m_initialTranslation = m_swerve.getState().Pose.getTranslation();
    // m_initialTranslation./
  
    m_initialRotationRadians = m_swerve.getState().Pose.getRotation().getRadians();
    m_targetRotationRadians = m_initialRotationRadians + rotation_radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(
      m_swerve.getState().Pose.getTranslation().getDistance(m_initialTranslation) - m_distanceMeters
    ) <= DISTANCE_DEADBAND;
  }
}
