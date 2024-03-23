// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PickUpPieceUntilSensorWithTimeout extends Command {
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;
  private final double m_timeoutSeconds;

  private final Timer m_timer = new Timer();
  public PickUpPieceUntilSensorWithTimeout(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, double timeoutSeconds) {
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;
    addRequirements(intake, indexer, shooter);

    m_timeoutSeconds = timeoutSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();

    m_intake.enableForward();
    m_indexer.enableForward();
    m_shooter.enableFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.disableForward();
    m_indexer.disableForward();
    m_shooter.disableFeeder();

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getWheelEntranceSensorTripped() || m_timer.get() >= m_timeoutSeconds;
  }
}
