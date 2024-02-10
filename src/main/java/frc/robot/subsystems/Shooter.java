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

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX top_motor = new TalonFX(10); // 13
  private final TalonFX bottom_motor = new TalonFX(9); // 14
 // TalonFX indexer_motor = new TalonFX(28);
  private GenericEntry rps_input;
  private GenericEntry rps_Index_Input;
  private final VelocityDutyCycle rps_DutyVelocity = new VelocityDutyCycle(0);
 
  // private final VisionSubsystem m_visionSubsystem;

  // public Shooter(VisionSubsystem vision) {
  //   m_visionSubsystem = vision;
  public Shooter(){

    top_motor.setNeutralMode(NeutralModeValue.Brake);
    bottom_motor.setNeutralMode(NeutralModeValue.Brake);
    // indexer_motor.setNeutralMode(NeutralModeValue.Brake);

    top_motor.setInverted(false);
    bottom_motor.setInverted(false);
    // indexer_motor.setInverted(true);
  
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0;
    slot0Configs.kV = 0.01;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    top_motor.getConfigurator().apply(slot0Configs, 0.050);
    bottom_motor.getConfigurator().apply(slot0Configs, 0.050);
    // indexer_motor.getConfigurator().apply(slot0Configs,0.050);
   
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter info");
    tab.add(this);
    tab.addDouble("FrontVelocity", () -> top_motor.getVelocity().getValue() );
    tab.addDouble("BackVelocity", () -> bottom_motor.getVelocity().getValue() );
    // tab.addDouble("IndexerVelocity", () -> indexer_motor.getVelocity().getValue() );

 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop() {
    top_motor.set(0);
    bottom_motor.set(0);
    // indexer_motor.set(0);
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
    top_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(value));
    bottom_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(value));
 //   indexer_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(value));
  }


  private static final double SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET = 3;
  // linear regression in the R programming language resulted in this equation
  public double calculateSpeedRPS(double distance_feet, double angle){
    distance_feet -= Shooter.SUBWOOFER_MIDDLE_APRIL_TAG_OFFSET_FEET;
    double speed_rpm = 2400 + (148.66 * distance_feet) + (12.08 * Math.abs(angle));
    SmartDashboard.putNumber("SHOOTER R/MINUTE", speed_rpm);
    // our data used revolutions per MINUTE, but TalonFX velocity uses revolutions per SECOND
    return speed_rpm / 60;
  }
}
