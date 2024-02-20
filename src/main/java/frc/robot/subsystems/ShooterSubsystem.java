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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_topMotor = new TalonFX(ShooterConstants.TOP_MOTOR_ID); // 13
  private final TalonFX m_bottomMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID); // 14
  private final TalonFX m_feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);

  private final VelocityDutyCycle m_request = new VelocityDutyCycle(ShooterConstants.TARGET_VELOCITY_RPS);
  private final MotionMagicDutyCycle m_indexerRequest = new MotionMagicDutyCycle(ShooterConstants.TARGET_INDEXER_POSITION);
 
  private boolean m_wantsToShoot = false;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter info");
  private final DoublePublisher m_topRPMPublisher = table.getDoubleTopic("TopRPM").publish();
  private final DoublePublisher m_bottomRPMPublisher = table.getDoubleTopic("BottomRPM").publish();

  // private final DoubleSubscriber m_targetRPMSubscriber = table.getDoubleTopic("TargetRPM").subscribe(4350);

  public ShooterSubsystem(){
    m_feederMotor.setPosition(0);

    m_topMotor.setInverted(false); //changed from true
    m_bottomMotor.setInverted(true);//changed from true
    // indexer_motor.setInverted(true);
  
    TalonFXConfiguration configs = new TalonFXConfiguration();

    Slot0Configs slot0Configs = configs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kA = 0.02;
    slot0Configs.kV = 0.52;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_feederMotor.setNeutralMode(NeutralModeValue.Brake);

    m_topMotor.getConfigurator().apply(configs, 0.050);
    m_bottomMotor.getConfigurator().apply(configs, 0.050);

    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));
  }

  @Override
  public void periodic() {
    m_topRPMPublisher.set(m_topMotor.getVelocity().getValueAsDouble() * 60);
    m_bottomRPMPublisher.set(m_bottomMotor.getVelocity().getValueAsDouble() * 60);

    // TODO: PUT THIS BACK (REMOVED FOR MANUAL TESTING)
    // if (m_wantsToShoot) {
    //   m_topMotor.setControl(m_request);
    //   m_bottomMotor.setControl(m_request);
    // } else {
    //   m_topMotor.disable();
    //   m_bottomMotor.disable();
    // }
  }
  // public void stop() {
  //   m_topMotor.disable();
  //   m_bottomMotor.disable();
  // }

  public InstantCommand m_toggleWantingToShoot = new InstantCommand(() -> {
    m_wantsToShoot = !m_wantsToShoot;
  }, this);
  public InstantCommand m_startWantingToShoot = new InstantCommand(() -> {
    m_wantsToShoot = true;
  }, this);
  public InstantCommand m_stopWantingToShoot = new InstantCommand(() -> {
    m_wantsToShoot = false;
  }, this);

  public void manualRunDeleteMe(double value) {
    m_topMotor.setControl(new DutyCycleOut(value));
  }
  public void manualRunRPMDeleteMe(double value) {
    m_topMotor.setControl(m_request.withVelocity(value / 60));
    // m_feederMotor.setControl(new DutyCycleOut(value));
  }
  // public void manualRunRPMDeleteMe() {
  //   manualRunRPMDeleteMe(m_targetRPMSubscriber.getAsDouble());
  // }
  public void manualStopDeleteMe() {
    m_topMotor.disable();
    // m_feederMotor.disable();
  }

  public void manualRunFeederDeleteMe(double value) {
    m_feederMotor.setControl(new DutyCycleOut(value));
  }
  public void manualStopFeederDeleteMe() {
    m_feederMotor.disable();
  }

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
  //   return speed_rpm / 60;
  // }
}
