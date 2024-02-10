// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AllianceAprilTagIDs;

public class VisionSubsystemNEW extends SubsystemBase {
  private AprilTagFieldLayout m_aprilTagFieldLayout = null;
  private final Dictionary<Integer, AprilTag> m_aprilTags = new Hashtable<>();

  private final CommandSwerveDrivetrain m_swerve;

  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(6);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);

  private final PhotonCamera m_frontCamera = new PhotonCamera("front_cam");
  private final PhotonCamera m_backCamera = new PhotonCamera("back_cam");
  private final PhotonCamera m_leftCamera = new PhotonCamera("left_cam");
  private final PhotonCamera m_rightCamera = new PhotonCamera("right_cam");

  // TODO: CHANGE THESE VALUES!!!!
  private final Transform3d m_frontRobotToCam = new Transform3d(
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
  // TODO: CHANGE THESE VALUES!!!!
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
  // TODO: CHANGE THESE VALUES!!!!
  private final Transform3d m_leftRobotToCam = new Transform3d(
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
  // TODO: CHANGE THESE VALUES!!!!
  private final Transform3d m_rightRobotToCam = new Transform3d(
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

  private PhotonPipelineResult m_frontResult, m_backResult, m_leftResult, m_rightResult;
  private final PhotonPoseEstimator m_frontEstimator, m_backEstimator, m_leftEstimator, m_rightEstimator;

  private final Hashtable<PhotonPipelineResult, PhotonPoseEstimator> m_pipeline_result_to_pose_estimator = new Hashtable<>();

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");

  private final DoublePublisher pitch_publisher = table.getDoubleTopic("Pitch").publish();
  private final DoublePublisher subwoofer_center_tag_distance = table.getDoubleTopic("subwoofer_center_tag_distance").publish();

  // private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

  public VisionSubsystemNEW(CommandSwerveDrivetrain swerve) {
    try {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
      // april_tag_field_layout = new AprilTagFieldLayout("deploy/2024-crescendo.json");

    } catch (Exception e){
      e.printStackTrace();
      DriverStation.reportError(e.getMessage(), true);

      m_swerve = null;

      m_frontEstimator = null;
      m_backEstimator = null;
      m_leftEstimator = null;
      m_rightEstimator = null;
      return;
    }

    m_swerve = swerve;

    for (AprilTag tag : m_aprilTagFieldLayout.getTags()) {
      m_aprilTags.put(tag.ID, tag);
    }

    // TODO: keep this?
    swerve.setVisionMeasurementStdDevs(VISION_STDS);
    
    m_frontEstimator = new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_frontCamera,
      m_frontRobotToCam
    );
    m_backEstimator = new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_backCamera,
      m_backRobotToCam
    );
    m_leftEstimator = new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_leftCamera,
      m_leftRobotToCam
    );
    m_rightEstimator = new PhotonPoseEstimator(
      m_aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_rightCamera,
      m_rightRobotToCam
    );

    m_pipeline_result_to_pose_estimator.put(m_frontResult, m_frontEstimator);
    m_pipeline_result_to_pose_estimator.put(m_backResult, m_backEstimator);
    m_pipeline_result_to_pose_estimator.put(m_leftResult, m_leftEstimator);
    m_pipeline_result_to_pose_estimator.put(m_rightResult, m_rightEstimator);
  }

  @Override
  public void periodic() {
    m_frontResult = m_frontCamera.getLatestResult();
    m_backResult = m_backCamera.getLatestResult();
    m_leftResult = m_leftCamera.getLatestResult();
    m_rightResult = m_rightCamera.getLatestResult();

    // huge thanks to https://www.chiefdelphi.com/u/Jus
    // https://www.chiefdelphi.com/t/multi-camera-setup-and-photonvisions-pose-estimator-seeking-advice/431154/2
    for (final PhotonPoseEstimator estimator : m_pipeline_result_to_pose_estimator.values()) {  
      final Optional<EstimatedRobotPose> optional_pose = estimator.update();
      if (optional_pose.isEmpty()) continue;
  
      final EstimatedRobotPose pose = optional_pose.get();
      m_swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    if (m_frontResult.hasTargets()) {
      Optional<PhotonTrackedTarget> subwoofer_center_tag = getTarget(AllianceAprilTagIDs.SUBWOOFER_CENTER, m_frontResult);
      if (subwoofer_center_tag.isPresent()) {
        PhotonTrackedTarget target = subwoofer_center_tag.get();
        pitch_publisher.set(target.getPitch());
        subwoofer_center_tag_distance.set(calculateDistanceToTargetMeters(target));
      }
    }
  }

  private double calculateDistanceToTargetMeters(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
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
}