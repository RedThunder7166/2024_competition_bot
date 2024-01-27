// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final double CAMERA_HEIGHT_METERS = 0.7493;
  private static final double TARGET_HEIGHT_METERS = 1.4351;
  public static double calculateDistanceToTargetMeters(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS, 
      TARGET_HEIGHT_METERS,
      0,
      Units.degreesToRadians(target.getPitch())
    );
  }

  private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private PhotonPipelineResult result;
  private boolean result_has_targets = false;
  private final Transform3d robot_to_cam = new Transform3d(new Translation3d(0.619125, 0.10795, 0.31115), new Rotation3d(0, Math.toRadians(10), 0));
  private PhotonPoseEstimator pose_estimator;

  // private double TURN_P = 0.02;
  // private double TURN_I = 0.001;
  // private double TURN_D = 0;
  private double TURN_P = 0.05;
  private double TURN_I = 0;
  private double TURN_D = 0;
  private final PIDController turn_controller = new PIDController(TURN_P, TURN_I, TURN_D);

  private double DRIVE_P = 0.01;
  private double DRIVE_I = 0.001;
  private double DRIVE_D = 0;
  private final PIDController drive_controller = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

  private ShuffleboardTab aimtarget_tab;
  private GenericEntry aimtarget_p;
  private GenericEntry aimtarget_i;
  private GenericEntry aimtarget_d;

  private double aimtarget_target_yaw = 0;

  private Optional<Double> latest_field_to_camera_rotation_z = Optional.empty();

  private final boolean TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD = true;
  public VisionSubsystem() {
    try {
      AprilTagFieldLayout april_tag_field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      pose_estimator = new PhotonPoseEstimator(april_tag_field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robot_to_cam);
    } catch(IOException e){
      e.printStackTrace();
    }

    if (TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD) {
      aimtarget_tab = Shuffleboard.getTab("AimTarget");
      aimtarget_p = aimtarget_tab.add("P", 0).getEntry();
      aimtarget_i = aimtarget_tab.add("I", 0).getEntry();
      aimtarget_d = aimtarget_tab.add("D", 0).getEntry();
      aimtarget_tab.addNumber("TARGET YAW", () -> aimtarget_target_yaw);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    result_has_targets = result.hasTargets();
    if (result_has_targets) {
      // System.out.println("Target found!");
      // List<PhotonTrackedTarget> targets = result.getTargets();
      // for (PhotonTrackedTarget target : targets) {
      //   System.out.println(target.getFiducialId());
      //   // double range = PhotonUtils.calculateDistanceToTargetMeters(
      //   //   CAMERA_HEIGHT_METERS, 
      //   //   TARGET_HEIGHT_METERS,
      //   //   0,
      //   //   Units.degreesToRadians(target.getPitch())
      //   // );
      //   // SmartDashboard.putNumber("Vision RANGE", range);
      //   System.out.println(target.getBestCameraToTarget());
      // }
    } else {
      // System.out.println("No targets.");
    }

    if (TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD) {
      TURN_P = aimtarget_p.getDouble(0);
      TURN_I = aimtarget_i.getDouble(0);
      TURN_D = aimtarget_d.getDouble(0);
      turn_controller.setPID(TURN_P, TURN_I, TURN_D);
      // System.out.println("P " + TURN_P + ", " + turn_controller.getP());
      // System.out.println("I " + TURN_I + ", " + turn_controller.getI());
      // System.out.println("D " + TURN_D + ", " + turn_controller.getD());
    }

    latest_field_to_camera_rotation_z = getFieldToCameraRotationZ();
    if (latest_field_to_camera_rotation_z.isPresent()) {
      SmartDashboard.putNumber("FieldToCameraRotationZ", latest_field_to_camera_rotation_z.get());
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return pose_estimator.update();
  }

  private Optional<Double> getFieldToCameraRotationZ(){
    Optional<EstimatedRobotPose> field_to_camera = getEstimatedGlobalPose();
    if (field_to_camera.isPresent()) {
      Rotation3d rotation = field_to_camera.get().estimatedPose.getRotation();
      return Optional.of(Math.toDegrees(rotation.getZ()));
    }
    return Optional.empty();
  }
  public Optional<Double> getLatestFieldToCameraRotationZ(){
    return latest_field_to_camera_rotation_z;
  }

  public double calculateTurnPower(){
    if (result_has_targets) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == 4) {
          aimtarget_target_yaw = target.getYaw();
          return turn_controller.calculate(aimtarget_target_yaw, 0);
        }
      }
    }
    return 0;
  }

  public boolean hasTarget(int id) {
    if (!result_has_targets) return false;
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == id) {
        return true;
      }
    }
    return false;
  }

  // public double calculateDrivePo

  // public double getTagSkew(int id) throws Exception {
  //   if (hasTarget(id)) {
  //     for (PhotonTrackedTarget target : result.getTargets()) {
  //       if (target.getFiducialId() == id) {
  //         return target.getSkew();
  //       }
  //     }
  //   }
  //   throw new Exception("NO TARGET");
  // }
  public Optional<Double> getTagDistance(int id) {
    if (hasTarget(id)) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == id) {
          return Optional.of(calculateDistanceToTargetMeters(target));
        }
      }
    }
    return Optional.empty();
  }
}
