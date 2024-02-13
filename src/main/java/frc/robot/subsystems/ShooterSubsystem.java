// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX m_topMotor = new TalonFX(ShooterConstants.TOP_MOTOR_ID); // 13
  private final TalonFX m_bottomMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID); // 14
  private final VelocityDutyCycle m_request = new VelocityDutyCycle(0);
 
  // private final VisionSubsystem m_visionSubsystem;

  // public Shooter(VisionSubsystem vision) {
  //   m_visionSubsystem = vision;
  public ShooterSubsystem(){

    m_topMotor.setNeutralMode(NeutralModeValue.Coast);
    m_bottomMotor.setNeutralMode(NeutralModeValue.Coast);

    m_topMotor.setInverted(false);
    m_bottomMotor.setInverted(false);
    // indexer_motor.setInverted(true);
  
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0;
    slot0Configs.kV = 0.01;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_topMotor.getConfigurator().apply(slot0Configs, 0.050);
    m_bottomMotor.getConfigurator().apply(slot0Configs, 0.050);
   
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter info");
    tab.add(this);
    tab.addDouble("FrontVelocity", () -> m_topMotor.getVelocity().getValue() );
    tab.addDouble("BackVelocity", () -> m_bottomMotor.getVelocity().getValue() ); 
  }

  @Override
  public void periodic() {
    
  }
  public void stop() {
    m_topMotor.stopMotor();
    m_bottomMotor.stopMotor();
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

  public void setVelocityRPS(double value) {
    m_topMotor.setControl(m_request.withSlot(0).withVelocity(value));
    m_bottomMotor.setControl(m_request.withSlot(0).withVelocity(value));
  }


  private static final double SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET = 3;
  // linear regression in the R programming language resulted in this equation
  public double calculateSpeedRPS(double distance_feet, double angle){
    distance_feet -= ShooterSubsystem.SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET;
    double speed_rpm = 2400 + (148.66 * distance_feet) + (12.08 * Math.abs(angle));
    SmartDashboard.putNumber("SHOOTER R/MINUTE", speed_rpm);
    // our data used revolutions per MINUTE, but TalonFX velocity uses revolutions per SECOND
    return speed_rpm / 60;
  }
}
