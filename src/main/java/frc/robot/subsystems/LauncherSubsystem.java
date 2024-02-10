// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private final TalonFX m_aimMotor = new TalonFX(LauncherConstants.AIM_MOTOR_ID);
  private final PositionVoltage m_aimMotorPositionRequest = new PositionVoltage(0).withSlot(0);

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("Launcher");
  
  public LauncherSubsystem() {
    m_aimMotor.setNeutralMode(NeutralModeValue.Brake);
    
    SoftwareLimitSwitchConfigs aim_softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    // aim_softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    // aim_softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = LauncherConstants.AIM_MOTOR_HIGHEST_ANGLE_POSITIONS;
    // aim_softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    // aim_softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = LauncherConstants.AIM_MOTOR_LOWEST_ANGLE_POSITIONS;

    aim_softwareLimitSwitchConfigs.ForwardSoftLimitEnable = false;
    aim_softwareLimitSwitchConfigs.ReverseSoftLimitEnable = false;

    m_aimMotor.getConfigurator().apply(aim_softwareLimitSwitchConfigs, 0.05);

    Slot0Configs aim_slot0configs = new Slot0Configs();
    aim_slot0configs.kP = 0.1; // 0.05
    aim_slot0configs.kI = 0;
    aim_slot0configs.kD = 0;

    m_aimMotor.getConfigurator().apply(aim_slot0configs, 0.05);

    m_shuffleBoardTab.addDouble("Aim Motor Positions", () -> m_aimMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopAim(){
    m_aimMotor.set(0);
  }

  public void setAimSpeed(double speed) {
    m_aimMotor.set(speed);
  }
  public void setAimPosition(double position){
    m_aimMotor.setControl(m_aimMotorPositionRequest.withPosition(position));
  }
}
