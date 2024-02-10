// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderSubsystem extends SubsystemBase {
  // private final TalonFX m_motor = new TalonFX(); // TODO: put an id here and possibly change to kraken
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public LoaderSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_motor.setControl(m_request.withPosition(100));
  }
}
