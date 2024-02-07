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
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final double CAMERA_HEIGHT_METERS = 0.635; // changed from 0.7874;
  private static final double TARGET_HEIGHT_METERS = 1.4351;
  private static final double CAMERA_PITCH_RADIANS = Math.toRadians(17);
  private static double calculateDistanceToTargetMeters(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS, 
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS, // changed from 0.16, 0.33, 0.31
      Units.degreesToRadians(target.getPitch())
    );
  }

  // private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private final PhotonCamera camera = new PhotonCamera("front_cam");
  // private final PhotonCamera front_camera = new PhotonCamera("front_cam");
  // private final PhotonCamera back_camera = new PhotonCamera("back_cam");
  // private final PhotonCamera left_camera = new PhotonCamera("left_cam");
  // private final PhotonCamera right_camera = new PhotonCamera("right_cam");

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

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
  private final DoubleEntry field_to_camera_rotation_z_entry = table.getDoubleTopic("FieldToCameraRotationZ").getEntry(0);
  private final DoubleEntry distance_to_apriltag_4_meters_entry = table.getDoubleTopic("DISTANCE TO APRILTAG 4 METERS").getEntry(0);

  private final StructPublisher<Pose2d> estimated_pose_publisher = table.getStructTopic("EstimatedPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> estimated_global_pose2d_publisher = table.getStructTopic("EstimatedGlobalPose2d", Pose2d.struct).publish();
  private final StructPublisher<Pose3d> estimated_global_pose3d_publisher = table.getStructTopic("EstimatedGlobalPose3d", Pose3d.struct).publish();


  private final StructPublisher<Pose2d> publisher = table
  .getStructTopic("MyPose", Pose2d.struct).publish();

  private final boolean TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD = true;
  public VisionSubsystem() {
    try {
      AprilTagFieldLayout april_tag_field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      pose_estimator = new PhotonPoseEstimator(april_tag_field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robot_to_cam);
    } catch(IOException e){
      e.printStackTrace();
    }

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();


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
      field_to_camera_rotation_z_entry.set(latest_field_to_camera_rotation_z.get());
    }

    Optional<Double> distance_meters = getTagDistance(4);
    if (distance_meters.isPresent()) {
      distance_to_apriltag_4_meters_entry.set(distance_meters.get());
    }

    Optional<Transform3d> estimated_pose = getMultiTagEstimatedPose();
    if (estimated_pose.isPresent()) {
      estimated_pose_publisher.set(new Pose2d(estimated_pose.get().getTranslation().toTranslation2d(), estimated_pose.get().getRotation().toRotation2d()));
      publisher.set(new Pose2d());
    }

    Optional<EstimatedRobotPose> estimated_global_pose = getEstimatedGlobalPose();
    if (estimated_global_pose.isPresent()) {
      estimated_global_pose2d_publisher.set(estimated_global_pose.get().estimatedPose.toPose2d());
      estimated_global_pose3d_publisher.set(estimated_global_pose.get().estimatedPose);
    }
  }

  // private class Transform3dSender implements Sendable {
  //   @Override
  //   public void initSendable(SendableBuilder builder) {
  //     // builder.setSmartDashboardType("");
  //     builder.
  //   }
  // }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (result_has_targets) {
      return pose_estimator.update(result);
    }
    return pose_estimator.update();
  }

  private Optional<Transform3d> getMultiTagEstimatedPose(){
    if(!result_has_targets) return Optional.empty();

    PNPResult pnp_result = result.getMultiTagResult().estimatedPose;
    if (!pnp_result.isPresent) return Optional.empty();

    return Optional.of(pnp_result.best);
  }

  private Optional<Double> getFieldToCameraRotationZ(){
    // Optional<EstimatedRobotPose> field_to_camera = getEstimatedGlobalPose();
    // if (field_to_camera.isPresent()) {
    //   Rotation3d rotation = field_to_camera.get().estimatedPose.getRotation();
    //   return Optional.of(Math.toDegrees(rotation.getZ()));
    // }
    // return Optional.empty()
  
    if (result_has_targets && result.getMultiTagResult().estimatedPose.isPresent) {
      return Optional.of(Math.toDegrees(result.getMultiTagResult().estimatedPose.best.getRotation().getZ()));
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
