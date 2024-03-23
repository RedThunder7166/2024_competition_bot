// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PickUpPieceUntilSensorWithTimeout;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 3 * Math.PI; //  1 rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver_joystick = new CommandXboxController(ControllerConstants.DRIVER_PORT); // My joystick
  private final CommandXboxController operator_joystick = new CommandXboxController(ControllerConstants.OPERATOR_PORT);
   private final XboxController joy = new XboxController(2);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  

/*Slew Rate limiting, Limits Acceleration of Directions */
private final SlewRateLimiter xLimiter = new SlewRateLimiter(15);

private final SlewRateLimiter yLimiter = new SlewRateLimiter(15);
// private final SlewRateLimiter rotLimiter = new SlewRateLimiter(.5);


  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.12).withRotationalDeadband(MaxAngularRate * 0.12) // Add a 5% deadband //used to be 10%
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  
  /* */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); //CHECKME this is wierd
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); //TODO Find a use in this for grabbing pieces

  /*Initialize Subsystems */
  private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain, logger);
  private final ClimberSubsystem m_climber = new ClimberSubsystem(drivetrain);
  private final LauncherSubsystem m_launcher = new LauncherSubsystem(m_vision);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem(m_shooter);
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LEDSubsystem m_led = new LEDSubsystem(m_intake, m_indexer, m_shooter); 
  private final DeflectorSubsystem m_deflector = new DeflectorSubsystem(m_shooter);
  
  private final ShuffleboardTab m_driverStationTab = Shuffleboard.getTab("DriverStation");
  
  private final InstantCommand m_startPickingUpPiece = new InstantCommand(() -> {
    m_intake.enableForward();
    m_indexer.enableForward();
    m_shooter.enableFeeder();
  }, m_intake, m_indexer, m_shooter);
  private final InstantCommand m_stopPickingUpPiece = new InstantCommand(() -> {
    m_intake.disableForward();
    m_indexer.disableForward();
    m_shooter.disableFeeder();
  }, m_intake, m_indexer, m_shooter);

    private final InstantCommand m_ReversePiece = new InstantCommand(() -> {
    m_intake.enableReverse();
    m_indexer.enableReverse();
    m_shooter.enableFeederReverse();
  }, m_intake, m_indexer, m_shooter);
  private final InstantCommand m_StopReversePiece = new InstantCommand(() -> {
    m_intake.disableReverse();
    m_indexer.disableReverse();
    m_shooter.disableFeederReverse();
  }, m_intake, m_indexer, m_shooter);

  private final SendableChooser<Alliance> m_allianceChooser = new SendableChooser<>();
   
  {
    // ...
    NamedCommands.registerCommand("StartPickingUpPiece", m_startPickingUpPiece);
    NamedCommands.registerCommand("StopPickingUpPiece", m_stopPickingUpPiece);

    NamedCommands.registerCommand("StartShooter", m_shooter.m_enableShooterCommand);
    NamedCommands.registerCommand("StopShooter", m_shooter.m_disableShooterCommand);

    NamedCommands.registerCommand("StartShooterReverse", m_shooter.m_enableShooterReverseCommand);
    NamedCommands.registerCommand("StopShooterReverse", m_shooter.m_disableShooterReverseCommand);

    NamedCommands.registerCommand("ReversePiece", m_ReversePiece);
    NamedCommands.registerCommand("StopReversePiece", m_StopReversePiece);

    
    NamedCommands.registerCommand("StartFeeder", new InstantCommand(() -> {
      m_shooter.enableFeeder();
    }, m_shooter));
    NamedCommands.registerCommand("StopFeeder", new InstantCommand(() -> {
      m_shooter.disableFeeder();
    }, m_shooter));
     NamedCommands.registerCommand("StartFeederReverse", new InstantCommand(() -> {
      m_shooter.enableFeederReverse();
    }, m_shooter));
     NamedCommands.registerCommand("StopFeederReverse", new InstantCommand(() -> {
      m_shooter.disableFeederReverse();
    }, m_shooter));
    NamedCommands.registerCommand("StopShooterAndFeeder", new InstantCommand(() -> {
      m_shooter.disableShooter();
      m_shooter.disableFeeder();
    }, m_shooter));





 
    NamedCommands.registerCommand("AimFromSubwoofer", new InstantCommand(() -> {
      AimLocation.setAimLocation(AimLocation.Subwoofer);
    }));
    NamedCommands.registerCommand("AimFromLoading", new InstantCommand(()-> {
      AimLocation.setAimLocation(AimLocation.Loading);
    }));
    NamedCommands.registerCommand("AimFromAmp", new InstantCommand(()-> {
      AimLocation.setAimLocation(AimLocation.Amp);
    }));
    NamedCommands.registerCommand("AimFromTrap", new InstantCommand(()-> {
      AimLocation.setAimLocation(AimLocation.Trap);
    }));
     NamedCommands.registerCommand("AutoAim", new InstantCommand(()-> {
      AimLocation.setAimLocation(AimLocation.AutoTarget);
    }));

   NamedCommands.registerCommand("LineUp", new ParallelRaceGroup(
    drivetrain.applyRequest(() -> {
      final Optional<Double> turn_power = m_vision.calculateTurnPower();
            return drive.withVelocityX(0)
                        .withVelocityY(0)
              .withRotationalRate((turn_power.orElse(0d)));// Drive counterclockwise with negative X (left)
    }), new WaitCommand(1)
   ));

   NamedCommands.registerCommand("PickUpPieceUntilSensor", new FunctionalCommand(() -> {
    m_intake.enableForward();
    m_indexer.enableForward();
    m_shooter.enableFeeder();
   }, () -> {}, (interrupt) -> {
    m_intake.disableForward();
    m_indexer.disableForward();
    m_shooter.disableFeeder();
   }, m_shooter::getWheelEntranceSensorTripped, m_intake, m_indexer, m_shooter));

   NamedCommands.registerCommand("PickUpPieceUntilSensorWithTimeout", 
    new PickUpPieceUntilSensorWithTimeout(m_intake, m_indexer, m_shooter)
   );
  }

  private boolean climber_up = false;
  private boolean climber_down = false;


  private boolean automatically_rotate = false;
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          final Optional<Double> turn_power = m_vision.calculateTurnPower();
          return drive.withVelocityX(xLimiter.calculate(((-driver_joystick.getLeftY()*MaxSpeed))))
          //  Math.pow(-driver_joystick.getLeftY() * MaxSpeed, 3)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(yLimiter.calculate(((-driver_joystick.getLeftX()*MaxSpeed) )))
              //Math.pow(-driver_joystick.getLeftX() * MaxSpeed, 3)) // Drive left with negative X (left)
            // use vision to rotate the robot when automatically_rotate is true; otherwise use joystick (see above)
            .withRotationalRate((automatically_rotate & turn_power.isPresent()) ? turn_power.get() : (-driver_joystick.getRightX() * MaxAngularRate));// Drive counterclockwise with negative X (left)
        }
      ));
    driver_joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    driver_joystick.a().whileTrue(Commands.startEnd(() -> {
      automatically_rotate = true;
    }, () -> {
      automatically_rotate = false;
    }));
    operator_joystick.x().whileTrue(Commands.startEnd(() -> {
      climber_up = true;
    }, () -> {
      climber_up = false;
    }, m_climber));
    operator_joystick.y().whileTrue(Commands.startEnd(() -> {
      climber_down = true;
    }, () -> {
      climber_down = false;
    }, m_climber));
    
    operator_joystick.rightBumper().whileTrue(Commands.startEnd(() -> {
      m_shooter.enableFeeder();
      m_intake.enableForward();
      m_indexer.enableForward();
    }, () -> {
    m_intake.disableForward();
    m_indexer.disableForward();
    m_shooter.disableFeeder();
  }, m_intake, m_indexer, m_shooter));
    
    operator_joystick.leftTrigger().onTrue(m_shooter.m_enableShooterReverseCommand);
    operator_joystick.leftTrigger().onFalse(m_shooter.m_disableShooterReverseCommand);

    operator_joystick.rightTrigger().onTrue(m_shooter.m_enableShooterCommand);
    operator_joystick.rightTrigger().onFalse(m_shooter.m_disableShooterCommand);

   
    operator_joystick.leftBumper().whileTrue(Commands.startEnd(() -> {
      m_shooter.enableFeederReverse();
      m_intake.enableReverse();
      m_indexer.enableReverse();
    }, () -> {
      m_shooter.disableFeederReverse();
      m_intake.disableReverse();
      m_indexer.disableReverse();
    }, m_shooter, m_intake, m_indexer));

   
   

    operator_joystick.start().onTrue(m_launcher.m_enableAimManualModeCommand);
    m_launcher.configureManualMode(operator_joystick::getLeftY);

    operator_joystick.povUp().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.Subwoofer);
    }, m_shooter, m_launcher));
    operator_joystick.povRight().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.Amp);
    }, m_shooter, m_launcher));
    operator_joystick.povDown().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.Trap);
    }, m_shooter, m_launcher));
    operator_joystick.povLeft().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.Speaker);
    }, m_shooter, m_launcher));

    operator_joystick.a().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.Loading);
    }, m_shooter, m_launcher));

    operator_joystick.b().onTrue(new InstantCommand(() -> {
      m_launcher.disableManualMode();
      AimLocation.setAimLocation(AimLocation.AutoTarget);
    }, m_shooter, m_launcher));

    m_climber.configureManualMode(() -> operator_joystick.getRightY()); //left arm


    final double climber_right_manual_mode_speed = 1;
    m_climber.configureRightManualMode( //right arm
      () -> climber_up ? climber_right_manual_mode_speed : 
        (climber_down ? -climber_right_manual_mode_speed : 0)
    );
  }
 
  {

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    m_driverStationTab.add(autoChooser);
  }

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    configureBindings();

    m_allianceChooser.onChange((Alliance a) -> {
      ReallyDumbAllianceColor.setAlliance(a);
    });

    m_allianceChooser.addOption("Blue", Alliance.Blue);
    m_allianceChooser.setDefaultOption("Red", Alliance.Red);

    m_driverStationTab.add(m_allianceChooser);
  }
      
  public void teleopInit() {
    m_launcher.teleopInit();
  }

  public void disabledInit() {
    m_indexer.disabledInit();
    m_intake.disabledInit();
    m_shooter.disabledInit();
  }

  public void autonomousExit() {
    drivetrain.seedFieldRelative();
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return autoChooser.getSelected();
    // return new TrackAprilTagCommand(drivetrain, m_visionSubsystem, drive, MaxAngularRate);
  }
}



