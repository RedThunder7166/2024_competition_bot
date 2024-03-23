// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimLocation;
import frc.robot.Constants.DeflectorConstants;

public class DeflectorSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_deflectorTab = Shuffleboard.getTab("Deflector");

  private final TalonFX m_motor = new TalonFX(DeflectorConstants.MOTOR_ID);
  private final TorqueCurrentFOC m_request = new TorqueCurrentFOC(DeflectorConstants.TARGET_AMPERES);

  // private final PIDController m_DeflectPIDController = new PIDController(
  //   0.1,
  //   0,
  //   0
  // );
  
  public DeflectorSubsystem() {
    final TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    
    m_motor.getConfigurator().apply(config);

    m_deflectorTab.addDouble("Motor Torque Current", this::getTorqueCurrent);
  }

  private double getTorqueCurrent() {
    return m_motor.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    if (AimLocation.getAimLocation() == AimLocation.Amp) {
      m_motor.setControl(m_request);
    } else {
      m_motor.disable();
    }
  }
}
