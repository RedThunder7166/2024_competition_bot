// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimLocation;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PickUpPieceUntilSensorWithTimeout extends Command {
  private final IntakeSubsystem m_intake;
  private final IndexerSubsystem m_indexer;
  private final ShooterSubsystem m_shooter;

  private static final double REVERSE_START_TIME = 2;
  private static final double REVERSE_STOP_TIME = 0.1;
  private static final double FORWARD_STOP_TIME = 0.3;

  private static final double FORWARD_START_TIME = REVERSE_START_TIME + REVERSE_STOP_TIME;
  private static final double END_TIME = FORWARD_START_TIME + FORWARD_STOP_TIME;

  private final Timer m_timer = new Timer();
  public PickUpPieceUntilSensorWithTimeout(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;
    addRequirements(intake, indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();

    AimLocation.setAimLocation(AimLocation.Loading);
    m_intake.enableForward();
    m_indexer.enableForward();
    m_shooter.enableFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    if (m_timer.get() >= FORWARD_START_TIME) {
      m_indexer.disableReverse();
      m_indexer.enableForward();

      m_shooter.disableFeederReverse();
      m_shooter.enableFeeder();
    } else if (m_timer.get() >= REVERSE_START_TIME) {
      m_intake.disableForward();

      m_indexer.disableForward();
      m_indexer.enableReverse();

      m_shooter.disableFeeder();
      m_shooter.enableFeederReverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.disableForward();

    m_indexer.disableForward();
    m_indexer.disableReverse();

    m_shooter.disableShooter();
    m_shooter.disableFeederReverse();

    m_shooter.disableFeeder();
    m_shooter.disableFeederReverse();

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getWheelEntranceSensorTripped() || m_timer.get() >= END_TIME;
  }
}
