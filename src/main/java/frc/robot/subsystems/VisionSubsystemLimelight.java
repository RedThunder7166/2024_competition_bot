// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AllianceColor;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.ReallyDumbAllianceColor;

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

  private static final double TURN_P = 0.015;
  private static final double TURN_I = 0;
  private static final double TURN_D = 0.003;
  private final PIDController turnController = new PIDController(TURN_P, TURN_I, TURN_D);

  private final CommandSwerveDrivetrain m_swerve;

  private final ShuffleboardTab m_driverStationTab = Shuffleboard.getTab("DriverStation");
  private boolean m_hasValidTurnPowerOutput = false;

  // private final GenericEntry m_turnPEntry = m_driverStationTab.add("TurnP", 0).getEntry();
  // private final GenericEntry m_turnIEntry = m_driverStationTab.add("TurnI", 0).getEntry();
  // private final GenericEntry m_turnDEntry = m_driverStationTab.add("TurnD", 0).getEntry();

  public VisionSubsystemLimelight(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;

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

    LimelightHelpers.SetRobotOrientation(
      "",
      m_swerve.getState().Pose.getRotation().getDegrees(),
      0, 0, 0, 0, 0
    );
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if(Math.abs(m_swerve.getPigeon2().getRate()) <= 720 && mt2.tagCount > 0) {
      m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
      m_swerve.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
    }
  }

  public void setPriorityID(int id) {
    m_priorityId = id;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("priorityid")
          .setNumber(id);
  }
  
  public void setVisionPriorityIDToSubwooferCenter(Alliance alliance) {
    setPriorityID(alliance == Alliance.Red ? AllianceColor.RED_SUBWOOFER_CENTER : AllianceColor.BLUE_SUBWOOFER_CENTER);
  }

  public boolean seesAprilTag() {
    return LimelightHelpers.getFiducialID("") != -1;
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

    // final double m = 249.295;
    // final double k = -1.61996;
    // final double x_zero = 0.0297632;
    // final double b = 151.212;
    final double m = 3.2272* Math.pow(10, 13);
    final double k = -1.43668;
    final double x_zero = -18.0102;
    final double b = 150.065;
    return (distance * (m / (1 + Math.pow(Math.E, -k * (distance - x_zero)))) + b) + 2;
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

  private Command ampPathCommand;

  public static final class TagPathCommand extends Command{
    private final int m_tag;
    private final VisionSubsystemLimelight m_vision;
    private boolean m_hasSeenTag = false;
    private Command m_pathCommand;

    public TagPathCommand(VisionSubsystemLimelight vision, int tag) {
      m_vision = vision;
      m_tag = tag;
    }

    @Override
    public void initialize() {
      m_hasSeenTag = false;
      m_vision.setPriorityID(m_tag);
    }

    @Override
    public void execute() {
      if (m_hasSeenTag) {
        // TODO: make this a transform3d that you add to plus that you give to the constructor
        final double degreeOffset = 180;    // FIXME: this needs to either be 0 or 180. might depend on color
        double translationOffset = 1;       // FIXME: in meters, this is how far our desired pose is from the amp tag
        translationOffset *= -1;            // FIXME: if this is dependant on color, then it depends on color; otherwise, it either will need to be here or not

        final List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
          m_vision.m_aprilTags.get(m_tag).pose.plus(new Transform3d(translationOffset, 0, 0, new Rotation3d(0,0,0))).toPose2d()
          .rotateBy(Rotation2d.fromDegrees(degreeOffset))
        );

        m_pathCommand = AutoBuilder.followPath(new PathPlannerPath(
          bezierPoints,
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
          new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
        ));
        m_pathCommand.schedule();
      } else {
        m_hasSeenTag = m_vision.seesAprilTag();
      }
    }

    @Override
    public void end(boolean interrupted) {
      if (m_pathCommand.isScheduled()) {
        m_pathCommand.cancel();
      }
    }

    @Override
    public boolean isFinished() {
      return m_pathCommand.isFinished();
    }
  }

  public Command getAmpPathCommand() {
    final int tag = ReallyDumbAllianceColor.getAlliance() == Alliance.Red ? AllianceColor.RED_AMP : AllianceColor.BLUE_AMP;

    return new TagPathCommand(this, tag);
  }
}
