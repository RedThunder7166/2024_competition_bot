// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystemLimelight extends SubsystemBase {
  private double TURN_P = 0.15;
  private double TURN_I = 0;
  private double TURN_D = 0;
  private final PIDController turn_controller = new PIDController(TURN_P, TURN_I, TURN_D);

  private final ShuffleboardTab m_driverStationTab = Shuffleboard.getTab("DriverStation");
  private boolean m_hasValidTurnPowerOutput = false;

  public VisionSubsystemLimelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_driverStationTab.addBoolean("Auto Turn Valid", () -> m_hasValidTurnPowerOutput);
  }

  public void setPriorityID(int id) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("priorityid")
          .setNumber(id);
  }

  public Optional<Double> calculateTurnPower() {
    if (LimelightHelpers.getFiducialID("") != -1) {
      double value = turn_controller.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("").getX());

      if (Math.abs(turn_controller.getPositionError()) < 0.4) {
        value = 0;
      }

      m_hasValidTurnPowerOutput = true;
      return Optional.of(value);
    }
    m_hasValidTurnPowerOutput = false;
    return Optional.empty();
  }
    
  public Optional<Double> calculateLauncherSpeakerAimPosition() {
    if (LimelightHelpers.getFiducialID("") != -1) {
      double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

      // how many degrees back is your limelight rotated from perfectly vertical?
      double limelightMountAngleDegrees = 0.0; 

      // distance from the center of the Limelight lens to the floor
      double limelightLensHeightInches = 23.0; 

      // distance from the target to the floor
      double goalHeightInches = 57.75;

      double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);

      //calculate distance
      double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

      return Optional.of(Units.inchesToMeters(distanceFromLimelightToGoalInches));
    }
    return Optional.empty();
  }
}
