// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private final TalonFX m_aimMotor = new TalonFX(LauncherConstants.AIM_MOTOR_ID);
  // private final PositionVoltage m_aimMotorPositionRequest = new PositionVoltage(0).withSlot(0);

  // TODO: look into dynamic motion magic
  private final MotionMagicDutyCycle m_goToLoadingPositionRequest = new MotionMagicDutyCycle(
    Utils.angleToEncoderUnits(LauncherConstants.AIM_CANCODER_LOADING_POSITION)
  );
  private final MotionMagicDutyCycle m_aimAtSpeakerRequest = new MotionMagicDutyCycle(0);

  private final CANcoder m_aimCANCoder = new CANcoder(LauncherConstants.AIM_CANCODER_ID);
  private boolean m_isAtLoadingPosition = false;
  private boolean m_wantsToLoad = false;

  private final VisionSubsystem m_vision;

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("Launcher");
  
  public LauncherSubsystem(VisionSubsystem vision) {
    m_vision = vision;

    final TalonFXConfiguration aim_config = new TalonFXConfiguration();
    
    final SoftwareLimitSwitchConfigs aim_sotware_limit_switch_configs = aim_config.SoftwareLimitSwitch;

    // aim_sotware_limit_switch_configs.ForwardSoftLimitEnable = true;
    // aim_sotware_limit_switch_configs.ForwardSoftLimitThreshold = LauncherConstants.AIM_MOTOR_HIGHEST_ANGLE_POSITIONS;
    // aim_sotware_limit_switch_configs.ReverseSoftLimitEnable = true;
    // aim_sotware_limit_switch_configs.ReverseSoftLimitThreshold = LauncherConstants.AIM_MOTOR_LOWEST_ANGLE_POSITIONS;

    aim_sotware_limit_switch_configs.ForwardSoftLimitEnable = false;
    aim_sotware_limit_switch_configs.ReverseSoftLimitEnable = false;
    
    // TODO: TUNE THIS
    final Slot0Configs aim_slot0configs = aim_config.Slot0;
    aim_slot0configs.kP = 0.1; // 0.05
    aim_slot0configs.kI = 0;
    aim_slot0configs.kD = 0;
    
    // TODO: TUNE THIS
    final var aim_motionmagic_configs = aim_config.MotionMagic;
    aim_motionmagic_configs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    aim_motionmagic_configs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    aim_motionmagic_configs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    m_aimMotor.setNeutralMode(NeutralModeValue.Brake);
    m_aimMotor.getConfigurator().apply(aim_config, 0.05);

    m_shuffleBoardTab.addDouble("Aim Motor Positions", () -> m_aimMotor.getPosition().getValueAsDouble());
    m_shuffleBoardTab.addDouble("Aim Moto CANCoder AbsolutePosition", () -> m_aimCANCoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    double value = m_aimCANCoder.getAbsolutePosition().getValueAsDouble();
    value = Math.abs(LauncherConstants.AIM_CANCODER_LOADING_POSITION - value);
    m_isAtLoadingPosition = value <= LauncherConstants.ALLOWABLE_CANCODER_ERROR;

    if (m_wantsToLoad) {
      m_aimMotor.setControl(m_goToLoadingPositionRequest);
    } else {
      aim();
    }
  }

  private void aim() {
    // TODO: manual override
    final Optional<Double> aim_angle_optional = m_vision.calculateLauncherSpeakerAimAngle();
    if (aim_angle_optional.isEmpty()) {
      m_aimMotor.disable();
      return;
    }

    m_aimMotor.setControl(m_aimAtSpeakerRequest.withPosition(
      Utils.angleToEncoderUnits(
        aim_angle_optional.get()
      )
    ));
  }

  public void teleopInit() {
    m_wantsToLoad = false;
  }

  public boolean getIsAtLoadingPosition() {
    return m_isAtLoadingPosition;
  }
  public boolean getWantsToLoad() {
    return m_wantsToLoad;
  }
  
  public void stopAimMotor(){
    m_aimMotor.disable();
  }
  
  public void setAimSpeed(double speed) {
    m_aimMotor.set(speed);
  }

  public InstantCommand m_toggleWantingToLoad = new InstantCommand(() -> {
    m_wantsToLoad = !m_wantsToLoad;
  }, this);
  public InstantCommand m_startWantingToLoad = new InstantCommand(() -> {
    m_wantsToLoad = true;
  }, this);
  public InstantCommand m_stopWantingToLoad = new InstantCommand(() -> {
    m_wantsToLoad = false;
  }, this);
  // public void setAimPosition(double position){
  //   m_aimMotor.setControl(m_aimMotorPositionRequest.withPosition(position));
  // }
}
