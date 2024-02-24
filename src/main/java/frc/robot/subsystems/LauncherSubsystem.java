// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LauncherConstants.AimPosition;

public class LauncherSubsystem extends SubsystemBase {
  private final TalonFX m_aimMotor = new TalonFX(LauncherConstants.AIM_MOTOR_ID);
  private final CANcoder m_aimCANCoder = new CANcoder(LauncherConstants.AIM_CANCODER_ID);

  // private final PositionVoltage m_aimMotorPositionRequest = new PositionVoltage(0).withSlot(0);

  // TODO: look into dynamic motion magic
  // private final MotionMagicDutyCycle m_goToLoadingPositionRequest = new MotionMagicDutyCycle(
  //   Utils.EncoderUnitsToAngle(LauncherConstants.AIM_CANCODER_LOADING_POSITION)
  // );
  // private final MotionMagicDutyCycle m_aimAtSpeakerRequest = new MotionMagicDutyCycle(0);

  private final MotionMagicDutyCycle m_aimRequest = new MotionMagicDutyCycle(0);
  private final DutyCycleOut m_aimManualRequest = new DutyCycleOut(0);

  private boolean m_isAtLoadingPosition = false;
  private boolean m_wantsToLoad = false;

  private final PIDController m_aimPIDController = new PIDController(
    0.15,
    0,
    0
  );
  private final double m_aimMaxSpeed = 1.5;

  private boolean m_manualAimState = false;
  private AimPosition m_aimTargetPosition = AimPosition.Loading;

  // private final VisionSubsystem m_vision;
  private final VisionSubsystemTEMPORARYDELETETHIS m_vision;
  private DoubleSupplier m_manual_aim_supplier;

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("Launcher");
  
  // public LauncherSubsystem(VisionSubsystem vision) {
  public LauncherSubsystem(VisionSubsystemTEMPORARYDELETETHIS vision) {
    m_vision = vision;

    final TalonFXConfiguration aim_config = new TalonFXConfiguration();
    
    final SoftwareLimitSwitchConfigs aim_sotware_limit_switch_configs = aim_config.SoftwareLimitSwitch;

    // aim_sotware_limit_switch_configs.ForwardSoftLimitEnable = true;
    // aim_sotware_limit_switch_configs.ForwardSoftLimitThreshold = LauncherConstants.AIM_MOTOR_HIGHEST_ANGLE_POSITIONS;
    // aim_sotware_limit_switch_configs.ReverseSoftLimitEnable = true;
    // aim_sotware_limit_switch_configs.ReverseSoftLimitThreshold = LauncherConstants.AIM_MOTOR_LOWEST_ANGLE_POSITIONS;

    aim_sotware_limit_switch_configs.ForwardSoftLimitEnable = false;
    aim_sotware_limit_switch_configs.ReverseSoftLimitEnable = false;
    
    // // TODO: TUNE THIS
    // final Slot0Configs aim_slot0configs = aim_config.Slot0;
    // // aim_slot0configs.kP = 0.01; // 0.05
    // aim_slot0configs.kP = 0.00001;
    // aim_slot0configs.kI = 0.0001;
    // aim_slot0configs.kD = 0;
    // // aim_slot0configs.kV = .1;
    // aim_slot0configs.kS = 0.2;
    
    // // TODO: TUNE THIS
    // final var aim_motionmagic_configs = aim_config.MotionMagic;
    
    // aim_motionmagic_configs.MotionMagicCruiseVelocity = 800; // Target cruise velocity of 80 rps
    // aim_motionmagic_configs.MotionMagicAcceleration = 1800; // Target acceleration of 160 rps/s (0.5 seconds)
    // aim_motionmagic_configs.MotionMagicJerk = 18000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    aim_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    aim_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    aim_config.CurrentLimits.SupplyCurrentLimit = 30;

    m_aimMotor.getConfigurator().apply(aim_config);

    m_shuffleBoardTab.addDouble("Aim Motor Position Degree", () -> m_aimMotor.getPosition().getValueAsDouble());
    m_shuffleBoardTab.addDouble("Aim Motor CANCoder AbsolutePosition Degree", this::getAimCANCoderAbsolutePositionDegrees);
    m_shuffleBoardTab.addDouble("Aim Motor CANCoder AbsolutePosition", this::getAimCANCoderAbsolutePosition);
    m_shuffleBoardTab.addDouble("Velocity", ()-> m_aimMotor.get() );
    m_shuffleBoardTab.addDouble("PID Error", () -> m_aimPIDController.getPositionError());

    m_shuffleBoardTab.addString("Aim Position", () -> m_aimTargetPosition.toString());
  }

  public void configureManualMode(DoubleSupplier supplier) {
    m_manual_aim_supplier = supplier;
  }

  public double getAimCANCoderAbsolutePositionDegrees() {
    return Math.toDegrees(m_aimCANCoder.getAbsolutePosition().getValueAsDouble());
  }
    public double getAimCANCoderAbsolutePosition() {
    return m_aimCANCoder.getAbsolutePosition().getValueAsDouble();
  }
//  public double getAimCANCoderAbsolutePositionDegrees() {
//     return Math.toDegrees(m_aimCANCoder.getAbsolutePosition().getValueAsDouble());
//   }
  @Override
  public void periodic() {
    // TODO: PUT THIS BACK (REMOVED FOR MANUAL TESTING)
    // double value = m_aimCANCoder.getAbsolutePosition().getValueAsDouble();
    // value = Math.abs(LauncherConstants.AIM_CANCODER_LOADING_POSITION - value);
    // m_isAtLoadingPosition = value <= LauncherConstants.ALLOWABLE_CANCODER_ERROR;

    // if (m_wantsToLoad) {
    //   m_aimMotor.setControl(m_goToLoadingPositionRequest);
    // } else {
    //   aim();
    // }
    m_aimMotor.setPosition(getAimCANCoderAbsolutePosition()*125*18);
    if (m_manualAimState) {
      manualAim(m_manual_aim_supplier.getAsDouble());
    } else {
      // m_aimMotor.setControl(m_aimRequest.withPosition(m_aimTargetPosition.position));
      double value = m_aimPIDController.calculate(
          m_aimMotor.getPosition().getValueAsDouble(), m_aimTargetPosition.position
      ) * m_aimMaxSpeed;
      if (Math.abs(m_aimPIDController.getPositionError()) > 0.5) {
        m_aimMotor.setControl(new DutyCycleOut(value));
      } else {
        m_aimMotor.disable();
      }
    }
  }

  // private void aim() {
  //   // TODO: manual override
  //   final Optional<Double> aim_position_optional = m_vision.calculateLauncherSpeakerAimPosition();
  //   if (aim_position_optional.isEmpty()) {
  //     m_aimMotor.disable();
  //     return;
  //   }

  //   m_aimMotor.setControl(m_aimAtSpeakerRequest.withPosition(
  //     // Utils.EncoderUnitsToAngle(
  //       aim_position_optional.get()
  //     // )
  //   ));
  // }

  private void manualAim(double value) {
    if (Math.abs(value) < LauncherConstants.MANUAL_AIM_DEADBAND) {
      m_aimMotor.disable();
      return;
    }

    m_aimMotor.setControl(m_aimManualRequest.withOutput(value));
  }

  public final InstantCommand m_enableAimManualModeCommand = new InstantCommand(() -> {
    m_manualAimState = true;
  }, this);

  private void setAimTarget(AimPosition position) {
    m_manualAimState = false;
    m_aimTargetPosition = position;
  }

  public final InstantCommand m_aimAtLoadingPositionCommand = new InstantCommand(() -> {
    setAimTarget(AimPosition.Loading);
  }, this);
  public final InstantCommand m_aimAtAmpCommand = new InstantCommand(() -> {
    setAimTarget(AimPosition.Amp);
  }, this);
  public final InstantCommand m_aimAtTrapCommand = new InstantCommand(() -> {
    setAimTarget(AimPosition.Trap);
  }, this);
  public final InstantCommand m_aimAtSpeakerCommand = new InstantCommand(() -> {
    setAimTarget(AimPosition.Speaker);
  }, this);
    
  // public void setAimPosition(double position) {
  //   m_aimMotor.setControl(m_aimRequest.withPosition(
  //     // Utils.EncoderUnitsToAngle(
  //       position
  //     // )
  //   ));
  // }

  public void teleopInit() {
    // m_wantsToLoad = false;
  }

  // public boolean getIsAtLoadingPosition() {
  //   return m_isAtLoadingPosition;
  // }
  // public boolean getWantsToLoad() {
  //   return m_wantsToLoad;
  // }
  
  // public void stopAimMotor(){
  //   m_aimMotor.disable();
  // }
  
  // public void setAimSpeed(double speed) {
  //   m_aimMotor.set(speed);
  // }

  // public InstantCommand m_toggleWantingToLoad = new InstantCommand(() -> {
  //   m_wantsToLoad = !m_wantsToLoad;
  // }, this);
  // public InstantCommand m_startWantingToLoad = new InstantCommand(() -> {
  //   m_wantsToLoad = true;
  // }, this);
  // public InstantCommand m_stopWantingToLoad = new InstantCommand(() -> {
  //   m_wantsToLoad = false;
  // }, this);

  // public void setAimPosition(double position){
  //   m_aimMotor.setControl(m_aimMotorPositionRequest.withPosition(position));
  // }

  // public void manualAimDeleteMe(double value) {
  //   m_aimMotor.setControl(new DutyCycleOut(value));
  // }
}
