// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystemNEW extends SubsystemBase {
  private AprilTagFieldLayout m_aprilTagFieldLayout = null;
  private final Dictionary<Integer, AprilTag> m_aprilTags = new Hashtable<>();

  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(6);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);

  private final PhotonCamera m_frontCamera = new PhotonCamera("front_cam");
  private final PhotonCamera m_backCamera = new PhotonCamera("back_cam");
  private final PhotonCamera m_leftCamera = new PhotonCamera("left_cam");
  private final PhotonCamera m_rightCamera = new PhotonCamera("right_cam");

  private PhotonPipelineResult m_frontResult, m_backResult, m_leftResult, m_rightResult;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");

  private final DoublePublisher pitch_publisher = table.getDoubleTopic("Pitch").publish();
  private final DoublePublisher tag4_distance = table.getDoubleTopic("tag4_distance").publish();

  public VisionSubsystemNEW() {
    try {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
      // april_tag_field_layout = new AprilTagFieldLayout("deploy/2024-crescendo.json");
    } catch (Exception e){
      e.printStackTrace();
      DriverStation.reportError(e.getMessage(), true);
    }

    for (AprilTag tag : m_aprilTagFieldLayout.getTags()) {
      m_aprilTags.put(tag.ID, tag);
    }
  }

  @Override
  public void periodic() {
    m_frontResult = m_frontCamera.getLatestResult();
    m_backResult = m_backCamera.getLatestResult();
    m_leftResult = m_leftCamera.getLatestResult();
    m_rightResult = m_rightCamera.getLatestResult();
    
    if (m_frontResult.hasTargets()) {
      Optional<PhotonTrackedTarget> target_4 = getTarget(4, m_frontResult);
      if (target_4.isPresent()) {
        PhotonTrackedTarget target = target_4.get();
        pitch_publisher.set(target.getPitch());
        tag4_distance.set(calculateDistanceToTargetMeters(target));
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