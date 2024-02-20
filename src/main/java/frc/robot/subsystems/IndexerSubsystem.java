// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(IndexerConstants.MOTOR_ID);
  private final VelocityDutyCycle m_request = new VelocityDutyCycle(IndexerConstants.TARGET_VELOCITY);

  private final LauncherSubsystem m_launcher;

  public IndexerSubsystem(LauncherSubsystem launcher) {
    m_launcher = launcher;

    m_motor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // TODO: PUT THIS BACK (REMOVED FOR MANUAL TESTING)
    // if (m_launcher.getIsAtLoadingPosition() && m_launcher.getWantsToLoad()) {
    //   m_motor.setControl(m_request);
    // } else {
    //   m_motor.disable();
    // }
  }

  public void manualRunDeleteMe(double value) {
    m_motor.setControl(new DutyCycleOut(value));
  }
  public void manualStopDeleteMe() {
    m_motor.disable();
  }
}