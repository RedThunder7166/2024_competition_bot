// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimLocation;
import frc.robot.Constants.DeflectorConstants;

public class DeflectorSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(DeflectorConstants.MOTOR_ID);
  // private final TorqueCurrentFOC m_request = new TorqueCurrentFOC(-DeflectorConstants.TARGET_AMPERES)
  //   .withMaxAbsDutyCycle(DeflectorConstants.MAX_ABS_DUTY_CYCLE);
  // private final TorqueCurrentFOC m_backwardRequest = new TorqueCurrentFOC(DeflectorConstants.TARGET_AMPERES)
  //   .withMaxAbsDutyCycle(DeflectorConstants.MAX_ABS_DUTY_CYCLE);

  private final ShooterSubsystem m_shooter;

  public DeflectorSubsystem(ShooterSubsystem shooter) {
    m_shooter = shooter;
    final TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    if (AimLocation.getAimLocation() == AimLocation.Amp && m_shooter.getWheelExitSensorTripped()) {
      // m_motor.setControl(m_request);
    } else {
      // m_motor.setControl(m_backwardRequest);
    }
  }
}
