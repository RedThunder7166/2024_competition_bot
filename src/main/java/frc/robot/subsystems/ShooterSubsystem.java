// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AimLocation;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_topMotor = new TalonFX(ShooterConstants.TOP_MOTOR_ID); // 13
  private final TalonFX m_bottomMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID); // 14
  private final TalonFX m_feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
  
  private final DigitalInput m_wheelEntranceSensor = new DigitalInput(ShooterConstants.WHEEL_ENTRANCE_SENSOR_ID);
  private final DigitalInput m_wheelExitSensor = new DigitalInput(ShooterConstants.WHEEL_EXIT_SENSOR_ID);

  // private final DutyCycleOut m_shooterRequest = new DutyCycleOut(AimLocation.getAimLocation().shooter_speed);
  private final VelocityDutyCycle m_shooterRequest = new VelocityDutyCycle(AimLocation.getAimLocation().shooter_speed_rps);
  private final VelocityDutyCycle m_shooterReverseRequest = new VelocityDutyCycle(-ShooterConstants.TARGET_SHOOTER_RPS);
  
  // private final DutyCycleOut m_feederRequest = new DutyCycleOut(1);
  private final VelocityDutyCycle m_feederRequest = new VelocityDutyCycle(ShooterConstants.TARGET_FEEDER_RPS);
  private final VelocityDutyCycle m_feederReverseRequest = new VelocityDutyCycle(-ShooterConstants.TARGET_FEEDER_RPS_BACKWARDS);
  
  private boolean m_shooterEnabled = false;
  private boolean m_shooterReverseEnabled = false;
  
  private boolean m_wheelEntranceSensorIsTripped = false;
  private boolean m_wheelExitSensorIsTripped = false;

  private boolean m_feederEnabled = false;
  private boolean m_feederReverseEnabled = false;
  
  private boolean m_shooterIsUpToSpeed = false;
  
  private final SequentialCommandGroup m_aimToLoadingCommand = new SequentialCommandGroup(
    new WaitCommand(ShooterConstants.AIM_TO_LOADING_DELAY_SECONDS),
    new InstantCommand(() -> {
        AimLocation.setAimLocation(AimLocation.Loading);
    })
  );

  

  private final ShuffleboardTab m_sensorTab = Shuffleboard.getTab("Sensors");
    
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter info");
  private final DoublePublisher m_topRPMPublisher = table.getDoubleTopic("TopRPM").publish();
  private final DoublePublisher m_bottomRPMPublisher = table.getDoubleTopic("BottomRPM").publish();
  private final DoublePublisher m_feederRPSPublisher = table.getDoubleTopic("FeederSpeed").publish();
  public ShooterSubsystem(){  
    TalonFXConfiguration top_configs = new TalonFXConfiguration();
    TalonFXConfiguration feeder_configs = new TalonFXConfiguration();
    
    Slot0Configs shooterSlot0Configs = top_configs.Slot0;
    shooterSlot0Configs.kS = 0.1;
    shooterSlot0Configs.kA = 0;
    shooterSlot0Configs.kV = .01;
    shooterSlot0Configs.kP = 0.049;
    shooterSlot0Configs.kI = 0;
    shooterSlot0Configs.kD = 0;

    Slot0Configs feederSlot0Configs = feeder_configs.Slot0;
    feederSlot0Configs.kS = 0;
    feederSlot0Configs.kV = 0.52;
    feederSlot0Configs.kP = 0.05;
    feederSlot0Configs.kI = 0;
    feederSlot0Configs.kD = 0;


    // clockwise is true
    // counterclockwise is false
    top_configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feeder_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    top_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feeder_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    top_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    top_configs.CurrentLimits.SupplyCurrentLimit = 80;

    feeder_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    feeder_configs.CurrentLimits.SupplyCurrentLimit = 50;

    m_topMotor.getConfigurator().apply(top_configs);
    m_bottomMotor.getConfigurator().apply(top_configs);
    m_feederMotor.getConfigurator().apply(feeder_configs);

    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));

    m_sensorTab.addBoolean("ShooterWheelEntrance", () -> m_wheelEntranceSensorIsTripped);
    m_sensorTab.addBoolean("ShooterWheelExit", () -> m_wheelExitSensorIsTripped);
  }
  
  @Override
  public void periodic() {
    m_wheelEntranceSensorIsTripped = Utils.isAllenBradleyTripped(m_wheelEntranceSensor);
    m_wheelExitSensorIsTripped = Utils.isAllenBradleyTripped(m_wheelExitSensor);

    // m_shooterIsUpToSpeed = m_topMotor.getVelocity().getValueAsDouble() >= ShooterConstants.SHOOTER_UP_TO_SPEED_THRESHOLD;

    final AimLocation aimLocation = AimLocation.getAimLocation();

    boolean overrideShooterLogic = false;
    if (DriverStation.isTeleopEnabled() && m_wheelExitSensorIsTripped) {
      if (aimLocation == AimLocation.Loading) {
        overrideShooterLogic = true;
        m_topMotor.setControl(m_shooterReverseRequest);
      } else if (aimLocation == AimLocation.Amp) {
         overrideShooterLogic = true;
        m_topMotor.setControl(m_shooterRequest);
      } else {
        m_aimToLoadingCommand.schedule();
      }
    }    

    m_topRPMPublisher.set((
      m_topMotor.getVelocity().getValueAsDouble()
    ));
    m_bottomRPMPublisher.set((
      m_bottomMotor.getVelocity().getValueAsDouble()
    ));
    m_feederRPSPublisher.set((
      m_feederMotor.getVelocity().getValueAsDouble()
    ));

    if (!overrideShooterLogic) {
      if (m_shooterEnabled) {
        m_topMotor.setControl(m_shooterRequest.withVelocity(aimLocation.shooter_speed_rps));
      } else if (m_shooterReverseEnabled) {
        m_topMotor.setControl(m_shooterReverseRequest);
      } else {
        m_topMotor.disable();
      }
    }

    if (m_feederReverseEnabled) {
      m_feederMotor.setControl(m_feederReverseRequest);
    } else if (m_feederEnabled) {
      m_feederMotor.setControl(m_feederRequest);
    } else {
      m_feederMotor.disable();
    }
  }

  public boolean getWheelEntranceSensorTripped() {
    return m_wheelEntranceSensorIsTripped;
  }

  public boolean getWheelExitSensorTripped() {
    return m_wheelExitSensorIsTripped;
  }

  public void disabledInit() {
    m_shooterEnabled = false;
    m_shooterReverseEnabled = false;
    m_feederEnabled = false;
    m_feederReverseEnabled = false;
    m_shooterIsUpToSpeed = false;
  }
  public InstantCommand m_enableShooterCommand = new InstantCommand(() -> {
    m_shooterEnabled = true;
  }, this);
  public InstantCommand m_disableShooterCommand = new InstantCommand(() -> {
    m_shooterEnabled = false;
  }, this);

  public InstantCommand m_enableShooterReverseCommand = new InstantCommand(() -> {
    m_shooterReverseEnabled = true;
  }, this);
  public InstantCommand m_disableShooterReverseCommand = new InstantCommand(() -> {
    m_shooterReverseEnabled = false;
  }, this);

  public void disableShooter() {
    m_shooterEnabled = false;
  }

  public void enableFeeder() {
    m_feederEnabled = true;
  }
  public void disableFeeder() {
    m_feederEnabled = false;
  }

  public void enableFeederReverse() {
    m_feederReverseEnabled = true;
  }
  public void disableFeederReverse() {
    m_feederReverseEnabled = false;
  }
}