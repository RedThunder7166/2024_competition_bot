// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX front_motor = new TalonFX(13);
  TalonFX back_motor = new TalonFX(14);
 // TalonFX indexer_motor = new TalonFX(28);
  private GenericEntry rps_input;
  private GenericEntry rps_Index_Input;
  private final VelocityDutyCycle rps_DutyVelocity = new VelocityDutyCycle(0);
 
  private final VisionSubsystem m_visionSubsystem;

  private int subwoofer_middle_tag_id = 4; // TODO: 4 if red alliance; 7 if blue alliance

  public Shooter(VisionSubsystem vision) {
    m_visionSubsystem = vision;

    front_motor.setNeutralMode(NeutralModeValue.Brake);
    back_motor.setNeutralMode(NeutralModeValue.Brake);
  //  indexer_motor.setNeutralMode(NeutralModeValue.Brake);

    front_motor.setInverted(false);
    back_motor.setInverted(false);
  //  indexer_motor.setInverted(true);
  
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0;
    slot0Configs.kV = 0.01;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    front_motor.getConfigurator().apply(slot0Configs, 0.050);
    back_motor.getConfigurator().apply(slot0Configs, 0.050);
  //  indexer_motor.getConfigurator().apply(slot0Configs,0.050);
   
    ShuffleboardTab tab = Shuffleboard.getTab("Fab test");
    tab.add(this);
    tab.addDouble("FrontVelocity", ()-> front_motor.getVelocity().getValue() );
    tab.addDouble("BackVelocity", ()-> back_motor.getVelocity().getValue() );
  //  tab.addDouble("IndexerVelocity", ()-> indexer_motor.getVelocity().getValue() );

 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop() {
    front_motor.set(0);
    back_motor.set(0);
 //   indexer_motor.set(0);
  }

  public void shoot(){
    Optional<Double> distance_meters = m_visionSubsystem.getTagDistance(subwoofer_middle_tag_id);
    if (distance_meters.isEmpty()){
      System.out.println("NO DISTNACE");
      stop();
      return;
    }
    System.out.println("YES DISTANCE");
    SmartDashboard.putNumber("DISTANCE TO APRILTAG", distance_meters.get());

    Optional<Double> field_to_camera_rotation_z = m_visionSubsystem.getLatestFieldToCameraRotationZ();
    if (field_to_camera_rotation_z.isEmpty()) {
      System.out.println("NO ROTATION");
      stop();
      return;
    }
    System.out.println("YES ROTATION");

    // our equation was modeled on data using feet, but photon vision returns meters, so we convert :D
    double speed = calculateSpeedRPS(distance_meters.get() * 3.28084 /* meters to feet */, field_to_camera_rotation_z.get());
    SmartDashboard.putNumber("SHOOTER R/SECOND", speed);
    setVelocityRPS(speed);
  }

  public void setVelocityRPS(double value) {
    front_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(-value));
    back_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(value));
 //   indexer_motor.setControl(rps_DutyVelocity.withSlot(0).withVelocity(value));
  }


  // linear regression in the R programming language resulted in this equation
  public double calculateSpeedRPS(double distance_feet, double angle){
    double speed_rpm = 2400 + (148.66 * distance_feet) + (12.08 * angle);
    SmartDashboard.putNumber("SHOOTER R/MINUTE", speed_rpm);
    // our data used revolutions per MINUTE, but TalonFX velocity uses revolutions per SECOND
    return speed_rpm / 60;
  }
}
