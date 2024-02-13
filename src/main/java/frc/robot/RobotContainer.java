// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystemOLD;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver_joystick = new CommandXboxController(ControllerConstants.DRIVER_PORT); // My joystick
  private final CommandXboxController operator_joystick = new CommandXboxController(ControllerConstants.OPERATOR_PORT);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final VisionSubsystemOLD m_visionSubsystem = new VisionSubsystemOLD();
  // private final Shooter m_shooter = new Shooter(m_visionSubsystem);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem(drivetrain);

  private boolean automatically_rotate = false;
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver_joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // use vision to rotate the robot when automatically_rotate is true; otherwise use joystick (see above)
            // .withRotationalRate(automatically_rotate ? (m_visionSubsystem.calculateTurnPower() * MaxAngularRate) : -joystick.getRightX() * MaxAngularRate) // booyah!
            // .withRotationalRate(((automatically_rotate && m_visionSubsystem.hasTarget(4)) ? m_visionSubsystem.calculateTurnPower() : -joystick.getRightX()) * MaxAngularRate)
        ));
    // drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));

    m_launcher.setDefaultCommand(new RunCommand(() -> {
      m_launcher.stopAim();
    }, m_launcher));

    m_shooter.setDefaultCommand(new RunCommand(() -> {
      m_shooter.stop();
      // m_shooter.shoot();
    }, m_shooter));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver_joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // controls for this are not final
    // operator_joystick.x().whileFalse(new TrackAprilTagCommand(drivetrain, m_visionSubsystem, drive, MaxAngularRate));
    driver_joystick.rightBumper().onTrue(new InstantCommand(() -> {
      automatically_rotate = false;
    }));
    driver_joystick.rightBumper().onFalse(new InstantCommand(() -> {
      automatically_rotate = true;
    }));

    driver_joystick.a().whileTrue(new RunCommand(() -> {
      m_launcher.setAimSpeed(1);
    }, m_launcher));
    driver_joystick.b().whileTrue(new RunCommand(() -> {
      m_launcher.setAimSpeed(-1);
    }, m_launcher));

    driver_joystick.y().whileTrue(new RunCommand(() -> {
      // m_launcher.setAimPosition(
    }, m_launcher));
    driver_joystick.x().whileTrue(new RunCommand(() -> {
      m_launcher.setAimPosition(-271);
    }, m_launcher));

    operator_joystick.a().whileTrue(new RunCommand(() -> {
      m_shooter.setVelocityRPS(80);
    }, m_launcher));
    operator_joystick.b().onTrue(new RunCommand(() -> {
      m_indexer.toggle();
    }, m_launcher));

    operator_joystick.povUp().onTrue(new InstantCommand(() -> {
      m_climber.setOutput(0.5);
    }, m_climber));
    operator_joystick.povDown().onTrue(new InstantCommand(() -> {
      m_climber.setOutput(-0.5);
    }, m_climber));
  }

  {
    // ...

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return new TrackAprilTagCommand(drivetrain, m_visionSubsystem, drive, MaxAngularRate);
  }
}



