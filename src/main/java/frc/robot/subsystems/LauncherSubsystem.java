// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimLocation;

import frc.robot.Constants.LauncherConstants;
public class LauncherSubsystem extends SubsystemBase {
  private final TalonFX m_aimMotor = new TalonFX(LauncherConstants.AIM_MOTOR_ID);
  private final CANcoder m_aimCANCoder = new CANcoder(LauncherConstants.AIM_CANCODER_ID);

  private final DutyCycleOut m_aimManualRequest = new DutyCycleOut(0);

  private boolean m_isAtLoadingPosition = false;
  private boolean m_wantsToLoad = false;

  private final PIDController m_aimPIDController = new PIDController(
    0.06,
    0,
    0
  );
  private final double m_aimMaxSpeed = 1.5;

  private boolean m_manualAimEnabled = false;

  private final VisionSubsystemLimelight m_vision;
  private DoubleSupplier m_left_stick_supplier;

  private final ShuffleboardTab m_shuffleBoardTab = Shuffleboard.getTab("Launcher");
  private final ShuffleboardTab m_driverStationTab = Shuffleboard.getTab("DriverStation");
  
  public LauncherSubsystem(VisionSubsystemLimelight vision) {
  // public LauncherSubsystem() {
    m_vision = vision;

    final TalonFXConfiguration aim_config = new TalonFXConfiguration();
    
    final SoftwareLimitSwitchConfigs aim_sotware_limit_switch_configs = aim_config.SoftwareLimitSwitch;

  
    aim_sotware_limit_switch_configs.ForwardSoftLimitEnable = false;
    aim_sotware_limit_switch_configs.ReverseSoftLimitEnable = false;
  
    aim_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    aim_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    aim_config.CurrentLimits.SupplyCurrentLimit = 30;

    m_aimMotor.getConfigurator().apply(aim_config);

    m_shuffleBoardTab.addDouble("Aim Motor Position Degree", this::getPosition);
    // m_shuffleBoardTab.addDouble("Aim Motor CANCoder AbsolutePosition", this::getAimCANCoderAbsolutePosition);
    m_shuffleBoardTab.addDouble("Velocity", ()-> m_aimMotor.get() );
    m_shuffleBoardTab.addDouble("PID Error", () -> m_aimPIDController.getPositionError());

    m_shuffleBoardTab.addString("AimLocation", () -> AimLocation.getAimLocation().name);
    m_driverStationTab.addString("AimLocation", () -> AimLocation.getAimLocation().name);
  }

  public void configureManualMode(DoubleSupplier left_supplier) {
    m_left_stick_supplier = left_supplier;
  }

  private double getPosition() {
    return m_aimCANCoder.getAbsolutePosition().getValueAsDouble() * 25 * 18;
  }

  private final DutyCycleOut m_automaticControl = new DutyCycleOut(0);
  @Override
  public void periodic() {
    // double value = m_aimCANCoder.getAbsolutePosition().getValueAsDouble();
    // value = Math.abs(LauncherConstants.AIM_CANCODER_LOADING_POSITION - value);
    // m_isAtLoadingPosition = value <= LauncherConstants.ALLOWABLE_CANCODER_ERROR;
  
    if (m_manualAimEnabled) {
      manualAim(m_left_stick_supplier.getAsDouble());
    } else {
      final double current_position = getPosition();
      final AimLocation aim_location = AimLocation.getAimLocation();
      final Optional<Double> auto_aim_optional = m_vision.calculateLauncherSpeakerAimPosition();
      double position = aim_location.auto_target ?
        (auto_aim_optional.isPresent() ? auto_aim_optional.get() : current_position)
        : aim_location.position;

      final double value = m_aimPIDController.calculate(current_position, position) * m_aimMaxSpeed;
      if (Math.abs(m_aimPIDController.getPositionError()) > 0.7) {
        m_aimMotor.setControl(m_automaticControl.withOutput(value));
      } else {
        m_aimMotor.disable();
      }
    }
  }

  private void manualAim(double left_stick) {
    if (Math.abs(left_stick) < LauncherConstants.MANUAL_AIM_DEADBAND) {
      m_aimMotor.disable();
      return; 
    }

    m_aimMotor.setControl(m_aimManualRequest.withOutput((left_stick / 2)));
  }

  public final InstantCommand m_enableAimManualModeCommand = new InstantCommand(() -> {
    m_manualAimEnabled = true;
  }, this);
  public void disableManualMode() {
    m_manualAimEnabled = false;
  } 

  public void teleopInit() {

  }

}
