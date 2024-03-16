// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Optional;
import java.util.Scanner;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants;
import frc.robot.Constants.AllianceColor;
import frc.robot.Constants.LauncherConstants;

import org.apache.commons.math3.stat.regression.SimpleRegression;

public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout m_aprilTagFieldLayout = null;
  private final Dictionary<Integer, AprilTag> m_aprilTags = new Hashtable<>();

  private final CommandSwerveDrivetrain m_swerve;
  private final Telemetry m_telemetry;

  // private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(6);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(10);

  // private double TURN_P = 0.02;
  // private double TURN_I = 0.001;
  // private double TURN_D = 0;
  private double TURN_P = 0.15;
  private double TURN_I = 0;
  private double TURN_D = 0;
  private final PIDController turn_controller = new PIDController(TURN_P, TURN_I, TURN_D);

  private final PhotonCamera m_frontCamera = new PhotonCamera("back_camera");
  // private final PhotonCamera m_backCamera = new PhotonCamera("back_camera");
//   private final PhotonCamera m_leftCamera = new PhotonCamera("left_cam");
  // private final PhotonCamera m_rightCamera = new PhotonCamera("right_cam");
  private final double ROBOT_CHASSIS_HEIGHT_OFF_GROUND_METERS = Units.inchesToMeters(1.5);
  // FIXME:rename from front to left front
  /*TODO: changed values, 
              from 
    7.75
    -11.28
    14
              To
    4
    11.25
    17.5

  */
  private final Transform3d m_frontRobotToCam = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(4),   // forward - backward
      Units.inchesToMeters(11.25), //  left   - right
      Units.inchesToMeters(16)     //   up    - down
    ),
    new Rotation3d(
      0,
      CAMERA_PITCH_RADIANS,
      0
    )
  );
  private final double FRONT_CAMERA_HEIGHT_OFF_GROUND_METERS = m_frontRobotToCam.getZ() + ROBOT_CHASSIS_HEIGHT_OFF_GROUND_METERS;

  // TODO: Add This in, Rename from Back to Right front
  private final Transform3d m_backRobotToCam = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(16),   // forward - backward
      0,                               //  left   - right
      Units.inchesToMeters(2)     //   up    - down
    ),
    new Rotation3d(
      0,
      Math.toRadians(18),
      0
    )
  );

  private PhotonPipelineResult m_frontResult;//, m_backResult;//, m_leftResult;//m_rightResult

  private final PhotonPoseEstimator m_frontEstimator;//, m_backEstimator;//, m_leftEstimator;//m_rightEstimator

  @FunctionalInterface
  private interface PhotonPipelineResultSupplier {
      /**
       * Gets a result.
       *
       * @return a result
       */
      PhotonPipelineResult getAsPhotonPipelineResult();
  }

  private final Hashtable<PhotonPipelineResultSupplier, PhotonPoseEstimator> m_pipeline_result_to_pose_estimator = new Hashtable<>();
  private final ArrayList<PhotonPoseEstimator> m_photonPoseEstimators = new ArrayList<>(3);
  private final Hashtable<PhotonPoseEstimator, EstimatedRobotPose> m_photonPoseEstimatorToEstimatedRobotPoseMap = new Hashtable<>();

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Vision");

  private final DoublePublisher m_pitchPublisher = m_table.getDoubleTopic("Pitch").publish();
  private final DoublePublisher m_subwooferCenterTagDistancePublisher = m_table.getDoubleTopic("subwoofer_center_tag_distance").publish();

  private final DoubleArrayPublisher m_estimatedPosePublisher = m_table.getDoubleArrayTopic("robotPose").publish();

  private final NetworkTable m_inMatchInformationTab = Constants.IN_MATCH_INFORMATION_TAB;
  private final BooleanPublisher m_autoTargetValidPublisher = m_inMatchInformationTab.getBooleanTopic("Auto Target Valid?").publish();
  private final NetworkTable m_DistanceToTarget = NetworkTableInstance.getDefault().getTable("Distance");
  private final DoublePublisher m_DistanceTarget = m_DistanceToTarget.getDoubleTopic("Meters").publish();
  private final DoublePublisher m_equationOutput = m_table.getDoubleTopic("EquationOutput").publish();
  // private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

  private final SimpleRegression m_distanceLaunchAngleRegression = new SimpleRegression();

  public VisionSubsystem(CommandSwerveDrivetrain swerve, Telemetry logger) {
    m_telemetry = logger;
    try {
      // m_aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
      m_aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");

    } catch (Exception e){
      e.printStackTrace();
      DriverStation.reportError(e.getMessage(), true);

      m_swerve = null;

      m_frontEstimator = null;
      // m_backEstimator = null;
    //   m_leftEstimator = null;
    //   m_rightEstimator = null;
      return;
    }

    try {
      File distanceLaunchAngleFile = new File(Filesystem.getDeployDirectory() + "/distance_launch_angle_regression.txt");
      Scanner reader = new Scanner(distanceLaunchAngleFile);
      while (reader.hasNextLine()) {
        String data = reader.nextLine();
        String[] split = data.split(";");
        m_distanceLaunchAngleRegression.addData(
          Double.valueOf(split[0]),
          Double.valueOf(split[1])
        );
      }
      reader.close();
    } catch (Exception e){
      e.printStackTrace();
      DriverStation.reportError(e.getMessage(), true);
    }

    m_swerve = swerve;

    for (AprilTag tag : m_aprilTagFieldLayout.getTags()) {
      m_aprilTags.put(tag.ID, tag);
    }
    
    m_frontEstimator = new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_frontCamera,
      m_frontRobotToCam
    );
    // m_backEstimator = new PhotonPoseEstimator(
    //   m_aprilTagFieldLayout,
    //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //   m_backCamera,
    //   m_backRobotToCam
    // );
    // m_leftEstimator = new PhotonPoseEstimator(
    //   m_aprilTagFieldLayout,
    //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //   m_leftCamera,
    //   m_leftRobotToCam
    // );
    // m_rightEstimator = new PhotonPoseEstimator(
    //   m_aprilTagFieldLayout,
    //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //   m_rightCamera,
    //   m_rightRobotToCam
    // );
    // m_rightEstimator = null;

    m_pipeline_result_to_pose_estimator.put(() -> m_frontResult, m_frontEstimator);
    // m_pipeline_result_to_pose_estimator.put(() -> m_backResult, m_backEstimator);
    // m_pipeline_result_to_pose_estimator.put(() -> m_leftResult, m_leftEstimator);
    // m_pipeline_result_to_pose_estimator.put(() -> m_rightResult, m_rightEstimator);

    m_photonPoseEstimators.add(m_frontEstimator);
    // m_photonPoseEstimators.add(m_backEstimator);
    // m_photonPoseEstimators.add(m_leftEstimator);
    // m_photonPoseEstimators.add(m_rightEstimator);
  }

  @Override
  public void periodic() {
    // get results from photonvision
    m_frontResult = m_frontCamera.getLatestResult();
    // m_backResult = m_backCamera.getLatestResult();
    // m_leftResult = m_leftCamera.getLatestResult();
    // m_rightResult = m_rightCamera.getLatestResult();

    // update swerve odometry
    // huge thanks to https://www.chiefdelphi.com/u/Jus
    // https://www.chiefdelphi.com/t/multi-camera-setup-and-photonvisions-pose-estimator-seeking-advice/431154/2
    Pose2d visionpose = null;
    for (final PhotonPoseEstimator estimator : m_pipeline_result_to_pose_estimator.values()) {  
    // for (final PhotonPoseEstimator estimator : m_photonPoseEstimators) {  
      final Optional<EstimatedRobotPose> optional_pose = estimator.update();
      if (optional_pose.isEmpty()) continue;
  
      final EstimatedRobotPose pose = optional_pose.get();
      m_photonPoseEstimatorToEstimatedRobotPoseMap.put(estimator, pose);

      // final Pose2d swerve_pose = m_swerve.getState().Pose;
      final Pose2d pose2d = pose.estimatedPose.toPose2d();
      visionpose = pose2d;
      m_estimatedPosePublisher.set(new double[] {visionpose.getX(),
        visionpose.getY(),
        visionpose.getRotation().getDegrees()}
      );
      
      m_swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    //   m_swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, VISION_STDS);
    }

    m_telemetry.telemeterize(m_swerve.getState());

    if (m_frontResult.hasTargets()) {
      Optional<PhotonTrackedTarget> subwoofer_center_tag = getTarget(AllianceColor.SUBWOOFER_CENTER, m_frontResult);
      if (subwoofer_center_tag.isPresent()) {
        PhotonTrackedTarget target = subwoofer_center_tag.get();
        m_pitchPublisher.set(target.getPitch());
        // // final Pose2d error = m_swerve.getState().Pose.relativeTo(m_aprilTags.get(target.getFiducialId()).pose.toPose2d());
        // final Pose2d swerve_pose = visionpose == null ? m_swerve.getState().Pose : visionpose;
        // final Pose2d target_pose = m_aprilTags.get(target.getFiducialId()).pose.toPose2d();
        // // final double distance = Math.sqrt( Math.pow(swerve_pose.getX() - target_pose.getX(), 2) + Math.pow(swerve_pose.getY() - target_pose.getY(), 2) );
        // final double distance = swerve_pose.getTranslation().getDistance(target_pose.getTranslation());
        // final double distance_old = calculateDistanceToTargetMeters(target, FRONT_CAMERA_HEIGHT_OFF_GROUND_METERS);
        // m_subwooferCenterTagDistancePublisher.set(Math.abs(distance - distance_old));

        m_subwooferCenterTagDistancePublisher.set(calculateDistanceToTargetMeters(target, FRONT_CAMERA_HEIGHT_OFF_GROUND_METERS));
      }
    }
  }

  private double calculateDistanceToTargetMeters(PhotonTrackedTarget target, double camera_height_meters) {
    return PhotonUtils.calculateDistanceToTargetMeters(
      camera_height_meters,
      m_aprilTags.get(target.getFiducialId()).pose.getZ(),
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(target.getPitch())
    );
  }

  public Optional<PhotonTrackedTarget> getTarget(int id, PhotonPipelineResult result) {
    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  private double distanceLaunchAngleCalculation(double distance) {
    return (100.622 / distance) - (-3.45926 * distance) + 113.535;
  }

  public Optional<Double> calculateLauncherSpeakerAimPosition() {
    if (m_frontResult.hasTargets()) {
      final Optional<PhotonTrackedTarget> target_optional = getTarget(AllianceColor.SUBWOOFER_CENTER, m_frontResult);
      if (target_optional.isPresent()) {
        final double distance = calculateDistanceToTargetMeters(target_optional.get(), FRONT_CAMERA_HEIGHT_OFF_GROUND_METERS);
        m_DistanceTarget.set(distance);
        // double position = m_distanceLaunchAngleRegression.predict(distance);
        double position = distanceLaunchAngleCalculation(distance);
        m_equationOutput.set(position);

        if (position <= LauncherConstants.AIM_MOTOR_LOWEST_POSITION || position >= LauncherConstants.AIM_MOTOR_HIGHEST_POSITION) {
          m_autoTargetValidPublisher.set(false);
          return Optional.empty();
        }
        m_autoTargetValidPublisher.set(true);
        return Optional.of(position);
      }
    }
    return Optional.empty();
  }

  public Optional<Double> calculateTurnPower() {
    final Optional<PhotonTrackedTarget> target_optional = getTarget(AllianceColor.SUBWOOFER_CENTER, m_frontResult);
    if (target_optional.isPresent()) {
      double value = turn_controller.calculate(target_optional.get().getYaw(), 0);
      // System.out.format("Value: %f | Error: %f\n", value, turn_controller.getPositionError());
      if (Math.abs(turn_controller.getPositionError()) < 0.5) {
        value = 0;
      }
      return Optional.of(value);
    }

    return Optional.empty();
  }
}