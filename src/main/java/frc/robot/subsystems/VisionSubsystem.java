// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private double TURN_P = 0.01;
  private double TURN_I = 0.001;
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

  private final boolean TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD = false;
  public VisionSubsystem() {
    if (TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD) {
      aimtarget_tab = Shuffleboard.getTab("AimTarget");
      aimtarget_p = aimtarget_tab.add("P", 0).getEntry();
      aimtarget_i = aimtarget_tab.add("I", 0).getEntry();
      aimtarget_d = aimtarget_tab.add("D", 0).getEntry();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    result_has_targets = result.hasTargets();
    if (result_has_targets) {
      System.out.println("Target found!");
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        System.out.println(target.getFiducialId());
        // double range = PhotonUtils.calculateDistanceToTargetMeters(
        //   CAMERA_HEIGHT_METERS, 
        //   TARGET_HEIGHT_METERS,
        //   0,
        //   Units.degreesToRadians(target.getPitch())
        // );
        // SmartDashboard.putNumber("Vision RANGE", range);
        System.out.println(target.getBestCameraToTarget());
      }
    } else {
      // System.out.println("No targets.");
    }

    if (TUNE_AIM_TARGET_PID_THROUGH_SHUFFLEBOARD) {
      TURN_P = aimtarget_p.getDouble(0);
      TURN_I = aimtarget_i.getDouble(0);
      TURN_D = aimtarget_d.getDouble(0);
      turn_controller.setPID(TURN_P, TURN_I, TURN_D);
      System.out.println("P " + TURN_P + ", " + turn_controller.getP());
      System.out.println("I " + TURN_I + ", " + turn_controller.getI());
      System.out.println("D " + TURN_D + ", " + turn_controller.getD());
    }
  }

  public double calculateTurnPower(){
    if (result_has_targets) {
      return turn_controller.calculate(result.getBestTarget().getYaw(), 0);
    }
    return 0;
  }

  // public double calculateDrivePo
}
