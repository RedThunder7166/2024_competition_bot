// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimLocation;
import frc.robot.Constants.JetEngineConstants;

public class JetEngineSubsystem extends SubsystemBase {
  private final TalonSRX m_motor = new TalonSRX(JetEngineConstants.MOTOR_ID);
  
  public JetEngineSubsystem() {
    final TalonSRXConfiguration config = new TalonSRXConfiguration();

    m_motor.configAllSettings(config);
    m_motor.setInverted(false);
    m_motor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    if (AimLocation.getAimLocation() == AimLocation.Trap) {
      m_motor.set(TalonSRXControlMode.PercentOutput, 1);
    } else {
      m_motor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }
}
