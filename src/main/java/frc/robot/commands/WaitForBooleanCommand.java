// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForBooleanCommand extends Command {
  private final BooleanSupplier m_supplier;

  /**
   * Waits for the boolean to be true
   * @param supplier
   */
  public WaitForBooleanCommand(BooleanSupplier supplier) {
    m_supplier = supplier;
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
    return m_supplier.getAsBoolean();
  }
}
