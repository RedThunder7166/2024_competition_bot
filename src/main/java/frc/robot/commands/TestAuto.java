// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.commands.ShootSubwooferCommand;


public class TestAuto extends Command {
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;
  private final LauncherSubsystem m_launcher;
  private final SwerveDrivetrain m_drive;
  /** Creates a new TestAuto. */
  public TestAuto(ShooterSubsystem shooter, LauncherSubsystem launcher, IndexerSubsystem indexer, SwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    m_shooter = shooter;
    m_launcher = launcher;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
