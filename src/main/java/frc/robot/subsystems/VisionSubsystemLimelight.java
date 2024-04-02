// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystemLimelight extends SubsystemBase {
  private AprilTagFieldLayout m_aprilTagFieldLayout = null;
  private final Dictionary<Integer, AprilTag> m_aprilTags = new Hashtable<>();

    // how many degrees back is your limelight rotated from perfectly vertical?
  private static final double limelightMountAngleDegrees = 15.0; 

  // distance from the center of the Limelight lens to the floor
  private static final double limelightLensHeightInches = 24.0 + (5/8); 

  // distance from the target to the floor
  private static final double goalHeightInches = 57.75;

  private int m_priorityId = 0;

  private static final double TURN_P = 0.011;
  private static final double TURN_I = 0;
  private static final double TURN_D = 0.002;
  private final PIDController turnController = new PIDController(TURN_P, TURN_I, TURN_D);

  private final ShuffleboardTab m_driverStationTab = Shuffleboard.getTab("DriverStation");
  private boolean m_hasValidTurnPowerOutput = false;

  // private final GenericEntry m_turnPEntry = m_driverStationTab.add("TurnP", 0).getEntry();
  // private final GenericEntry m_turnIEntry = m_driverStationTab.add("TurnI", 0).getEntry();
  // private final GenericEntry m_turnDEntry = m_driverStationTab.add("TurnD", 0).getEntry();

  public VisionSubsystemLimelight() {
    m_driverStationTab.addBoolean("Auto Turn Valid", () -> m_hasValidTurnPowerOutput);

    try {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");

      for (AprilTag tag : m_aprilTagFieldLayout.getTags()) {
        m_aprilTags.put(tag.ID, tag);
      }
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError(e.getMessage(), true);

      return;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPriorityID(int id) {
    m_priorityId = id;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("priorityid")
          .setNumber(id);
  }

  public Optional<Double> calculateTurnPower() {
    if (LimelightHelpers.getFiducialID("") != -1) {
      // turnController.setPID(m_turnPEntry.getDouble(0), m_turnIEntry.getDouble(0), m_turnDEntry.getDouble(0));
      final double tx = LimelightHelpers.getTX("");
      double value = turnController.calculate(tx);

      SmartDashboard.putNumber("TurnPower", value);
      SmartDashboard.putNumber("TurnError", turnController.getPositionError());

      // if (Math.abs(turnController.getPositionError()) < 5) {
      //   // value = 0;
      //   value = Math.copySign(0.2, -tx);
      // }

      m_hasValidTurnPowerOutput = true;
      return Optional.of(value);
    }
    m_hasValidTurnPowerOutput = false;
    return Optional.empty();
  }

  public Optional<Double> calculateTurnDegrees() {
    if (LimelightHelpers.getFiducialID("") != -1) {
      final double tx = LimelightHelpers.getTX("");

      return Optional.of(tx);
    }
    return Optional.empty();
  }

  private double distanceLaunchAngleCalculation(double distance) {
    // return (-42.4977/(1+Math.pow(Math.E, (-2.20952 * ( distance - 1.725))))+199.689);

    final double m = 249.295;
    final double k = -1.61996;
    final double x_zero = 0.0297632;
    final double b = 151.212;
    return distance * (m / (1 + Math.pow(Math.E, -k * (distance - x_zero)))) + b;
  }
    
  public Optional<Double> calculateLauncherSpeakerAimPosition() {
    if (LimelightHelpers.getFiducialID("") != -1) {
      double targetOffsetAngle_Vertical = LimelightHelpers.getTY("");

      double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + targetOffsetAngle_Vertical);

      //calculate distance
      double distanceFromLimelightToGoalInches = (Units.metersToInches(m_aprilTags.get(m_priorityId).pose.getZ()) - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
      distanceFromLimelightToGoalInches -= VisionConstants.DISTANCE_BETWEEN_CAMERA_AND_FRONT_OF_ROBOT;
      double distanceMeters = Units.inchesToMeters(distanceFromLimelightToGoalInches);

      SmartDashboard.putNumber("LimelightDistanceMeters", distanceMeters);

      double position = distanceLaunchAngleCalculation(distanceMeters);

      if (position <= LauncherConstants.AIM_MOTOR_LOWEST_POSITION || position >= LauncherConstants.AIM_MOTOR_HIGHEST_POSITION) {
        return Optional.empty();
      }

      return Optional.of(position);
    }
    return Optional.empty();
  }
}
