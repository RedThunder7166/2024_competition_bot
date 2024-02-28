// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimLocation;
import frc.robot.Utils;
// import frc.robot.Constants.LauncherConstants.AimPosition;
// import frc.robot.Constants.ShooterConstants.AimSpeed;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_topMotor = new TalonFX(ShooterConstants.TOP_MOTOR_ID); // 13
  private final TalonFX m_bottomMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID); // 14
  private final TalonFX m_feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
  private final DigitalInput m_breakBeamSensor = new DigitalInput(ShooterConstants.BREAK_BEAM_SENSOR_ID);
  
  // private final DutyCycleOut m_shooterRequest = new DutyCycleOut(ShooterConstants.AimSpeed.Speaker.speed);
  private final DutyCycleOut m_shooterRequest = new DutyCycleOut(AimLocation.getAimLocation().shooter_speed);
  private final VelocityDutyCycle m_shooterReverseRequest = new VelocityDutyCycle(-ShooterConstants.TARGET_SHOOTER_RPS);
  
  private final DutyCycleOut m_feederRequest = new DutyCycleOut(1);
  // private final DutyCycleOut m_feederRequest = new DutyCycleOut(AimLocation.getAimLocation().feeder_speed);
  private final VelocityDutyCycle m_feederReverseRequest = new VelocityDutyCycle(-ShooterConstants.TARGET_FEEDER_RPS);
  
  private boolean m_shooterState = false;
  private boolean m_shooterReverseState = false;
  
  private boolean m_feederState = false;
  private boolean m_feederReverseState = false;
  
  private boolean m_shooterIsUpToSpeed = false;
  
  // private ShooterConstants.AimSpeed m_speed = ShooterConstants.AimSpeed.Speaker;
  
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter info");
  private final DoublePublisher m_topRPMPublisher = table.getDoubleTopic("TopRPM").publish();
  private final DoublePublisher m_bottomRPMPublisher = table.getDoubleTopic("BottomRPM").publish();
  private final BooleanPublisher m_breakBeamSensorPublisher = table.getBooleanTopic("BreakBeamSensor").publish();
  
  // private final DoubleSubscriber m_targetRPMSubscriber = table.getDoubleTopic("TargetRPM").subscribe(4350);

  public ShooterSubsystem(){  
    TalonFXConfiguration top_configs = new TalonFXConfiguration();
    TalonFXConfiguration feeder_configs = new TalonFXConfiguration();
    
    Slot0Configs slot0Configs = top_configs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kA = 0.02;
    slot0Configs.kV = 0.52;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    feeder_configs.Slot0 = slot0Configs;

    // clockwise is true
    // counterclockwise is false
    top_configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feeder_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    top_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feeder_configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    top_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    top_configs.CurrentLimits.SupplyCurrentLimit = 80;

    feeder_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    feeder_configs.CurrentLimits.SupplyCurrentLimit = 10;

    m_topMotor.getConfigurator().apply(top_configs);
    m_bottomMotor.getConfigurator().apply(top_configs);
    m_feederMotor.getConfigurator().apply(feeder_configs);

    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    m_topRPMPublisher.set(Utils.secondToMinute(
      m_topMotor.getVelocity().getValueAsDouble()
    ));
    m_bottomRPMPublisher.set(Utils.secondToMinute(
      m_bottomMotor.getVelocity().getValueAsDouble()
    ));
    m_breakBeamSensorPublisher.set(m_breakBeamSensor.get());

    m_shooterIsUpToSpeed = m_topMotor.getVelocity().getValueAsDouble() >= ShooterConstants.SHOOTER_UP_TO_SPEED_THRESHOLD;

    if (m_shooterState) {
      // m_topMotor.setControl(m_shooterRequest.withOutput(m_speed.speed));
      m_topMotor.setControl(m_shooterRequest.withOutput(AimLocation.getAimLocation().shooter_speed));
    } else if (m_shooterReverseState) {
      m_topMotor.setControl(m_shooterReverseRequest);
    } else {
      m_topMotor.disable();
    }

    if (m_feederReverseState) {
      m_feederMotor.setControl(m_feederReverseRequest);
    } else if (m_shooterIsUpToSpeed || m_feederState) {
      // m_feederMotor.setControl(m_feederRequest.withOutput(m_speed.feeder_speed));
      m_feederMotor.setControl(m_feederRequest);
      // m_feederMotor.setControl(m_feederRequest.withOutput(AimLocation.getAimLocation().feeder_speed));
    } else {
      m_feederMotor.disable();
    }
  }
  // public void stop() {
  //   m_topMotor.disable();
  //   m_bottomMotor.disable();
  // }

  public void disabledInit() {
    m_shooterState = false;
    m_shooterReverseState = false;
    m_feederState = false;
    m_feederReverseState = false;
    m_shooterIsUpToSpeed = false;
  }

  public InstantCommand m_enableShooterCommand = new InstantCommand(() -> {
    m_shooterState = true;
  }, this);
  public InstantCommand m_disableShooterCommand = new InstantCommand(() -> {
    m_shooterState = false;
  }, this);

  public InstantCommand m_enableShooterReverseCommand = new InstantCommand(() -> {
    m_shooterReverseState = true;
  }, this);
  public InstantCommand m_disableShooterReverseCommand = new InstantCommand(() -> {
    m_shooterReverseState = false;
  }, this);

  // public InstantCommand m_enableFeederCommand = new InstantCommand(() -> {
  //   m_feederState = true;
  // }, this);
  // public InstantCommand m_disableFeederCommand = new InstantCommand(() -> {
  //   m_feederState = false;
  // }, this);

  public void disableShooter() {
    m_shooterState = false;
  }

  public void enableFeeder() {
    m_feederState = true;
  }
  public void disableFeeder() {
    m_feederState = false;
  }

  public void enableFeederReverse() {
    m_feederReverseState = true;
  }
  public void disableFeederReverse() {
    m_feederReverseState = false;
  }

  // public final InstantCommand m_speedLoadingPositionCommand = new InstantCommand(() -> {
  //   m_speed = AimSpeed.Loading;
  // }, this);
  // public final InstantCommand m_speedAmpCommand = new InstantCommand(() -> {
  //   m_speed = AimSpeed.Amp;
  // }, this);
  // public final InstantCommand m_speedTrapCommand = new InstantCommand(() -> {
  //   m_speed = AimSpeed.Trap;
  // }, this);
  // public final InstantCommand m_speedSpeakerCommand = new InstantCommand(() -> {
  //   m_speed = AimSpeed.Speaker;
  // }, this);

  // public void manualRunDeleteMe(double value) {
  //   m_topMotor.setControl(new DutyCycleOut(value));
  // }
  // public void manualRunRPMDeleteMe(double value) {
  //   m_topMotor.setControl(m_shooterRequest.withVelocity(Utils.minuteToSecond(value)));
  //   // m_feederMotor.setControl(new DutyCycleOut(value));
  // }
  // // public void manualRunRPMDeleteMe() {
  // //   manualRunRPMDeleteMe(m_targetRPMSubscriber.getAsDouble());
  // // }
  // public void manualStopDeleteMe() {
  //   m_topMotor.disable();
  //   // m_feederMotor.disable();
  // }

  // public void manualRunFeederDeleteMe(double value) {
  //   m_feederMotor.setControl(new DutyCycleOut(value));
  // }
  // public void manualStopFeederDeleteMe() {
  //   m_feederMotor.disable();
  // }

  // public void shoot(){
  //   Optional<Double> distance_meters = m_visionSubsystem.getTagDistance(AllianceAprilTagIDs.SUBWOOFER_CENTER);
  //   if (distance_meters.isEmpty()){
  //     System.out.println("NO DISTANCE");
  //     stop();
  //     return;
  //   }
  //   System.out.println("YES DISTANCE");

  //   Optional<Double> field_to_camera_rotation_z = m_visionSubsystem.getLatestFieldToCameraRotationZ();
  //   if (field_to_camera_rotation_z.isEmpty()) {
  //     System.out.println("NO ROTATION");
  //     stop();
  //     return;
  //   }
  //   System.out.println("YES ROTATION");

  //   // our equation was modeled on data using feet, but photon vision returns meters, so we convert :D
  //   double speed = calculateSpeedRPS(distance_meters.get() * 3.28084 /* meters to feet */, field_to_camera_rotation_z.get());
  //   SmartDashboard.putNumber("SHOOTER R/SECOND", speed);
  //   setVelocityRPS(speed);
  // }

  // public void setVelocityRPS(double value) {
  //   m_topMotor.setControl(m_request.withSlot(0).withVelocity(value));
  //   m_bottomMotor.setControl(m_request.withSlot(0).withVelocity(value));
  // }


  // private static final double SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET = 3;
  // // linear regression in the R programming language resulted in this equation
  // public double calculateSpeedRPS(double distance_feet, double angle){
  //   distance_feet -= ShooterSubsystem.SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET;
  //   double speed_rpm = 2400 + (148.66 * distance_feet) + (12.08 * Math.abs(angle));
  //   SmartDashboard.putNumber("SHOOTER R/MINUTE", speed_rpm);
  //   // our data used revolutions per MINUTE, but TalonFX velocity uses revolutions per SECOND
  //   return Utils.secondToMinute(speed_rpm);
  // }
}