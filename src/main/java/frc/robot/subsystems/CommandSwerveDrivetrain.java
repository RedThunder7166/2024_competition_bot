package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.DynamicTag;
import frc.robot.generated.TunerConstants;
// import frc.robot.sysidutils.ModifiedSignalLogger;
//import frc.robot.sysidutils.SwerveVoltageRequest;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        configureMotors();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        configureMotors();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        /*This is How You Set Supply Limits Along with Stator Limits */
        // for(int i = 0; i < 4; ++i) {
        //     CurrentLimitsConfigs newConfig = new CurrentLimitsConfigs();
        //     StatusCode returnCode = StatusCode.StatusCodeNotInitialized;
        //     int maxRetries = 5;
        //     do {
        //         returnCode = getModule(i).getDriveMotor().getConfigurator().refresh(newConfig);
        //     } while (!returnCode.isOK() && maxRetries-- > 0);

        //     newConfig.SupplyCurrentLimit = 1;
        //     newConfig.SupplyCurrentLimitEnable = true;
        //     maxRetries = 5;
        //     do {
        //         returnCode = getModule(i).getDriveMotor().getConfigurator().apply(newConfig);
        //     } while (!returnCode.isOK() && maxRetries-- > 0);
            
        // }
    }

    private void configureMotors() {
    
    }
    // @Override
    public void seedFieldRelative() {
        super.seedFieldRelative();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
    
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(4, 0, 0),
                                            new PIDConstants(4, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            () -> {
                // return ReallyDumbAllianceColor.getAlliance() == DriverStation.Alliance.Red;
                return DynamicTag.alliance == Alliance.Red;
            },
            this); // Subsystem for requirements
    }

    public void autoExit() {
        // m_fieldRelativeOffset = Rotation2d.fromDegrees(ReallyDumbAllianceColor.getAlliance() == Alliance.Red ? 180 : 0);
        m_fieldRelativeOffset = Rotation2d.fromDegrees(DynamicTag.alliance == Alliance.Red ? 180 : 0);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }
    public void setModuleStates(SwerveModuleState[] speeds) {
        for (int index = 0; index < 4; index++) {
        Modules[index].apply(speeds[index], DriveRequestType.Velocity);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // private final DoublePublisher[] module_publishers = {
    //     table.getDoubleTopic("Module0Drive StatorCurrent").publish(),
    //     table.getDoubleTopic("Module0Steer StatorCurrent").publish(),
    //     table.getDoubleTopic("Module1Drive StatorCurrent").publish(),
    //     table.getDoubleTopic("Module1Steer StatorCurrent").publish(),
    //     table.getDoubleTopic("Module2Drive StatorCurrent").publish(),
    //     table.getDoubleTopic("Module2Steer StatorCurrent").publish(),
    //     table.getDoubleTopic("Module3Drive StatorCurrent").publish(),
    //     table.getDoubleTopic("Module3Steer StatorCurrent").publish(),
    //     table.getDoubleTopic("Module4Drive StatorCurrent").publish(),
    //     table.getDoubleTopic("Module4Steer StatorCurrent").publish(),
    // };

    // private final DoublePublisher module0drive_statorcurrent = table.getDoubleTopic("Module0Drive StatorCurrent").publish();
    // private final DoublePublisher module0drive_velocity = table.getDoubleTopic("Module0Drive Velocity").publish();


    @Override
    public void periodic() {
        // int publisher_index = 0;
        // // amperage of each motor
        // for (int index = 0; index < 4; index++) {
        //     SwerveModule module = getModule(index);
            
        //     module_publishers[publisher_index].set(module.getDriveMotor().getStatorCurrent().getValueAsDouble());
        //     module_publishers[publisher_index + 1].set(module.getSteerMotor().getStatorCurrent().getValueAsDouble());
        //     publisher_index += 2;
        // }

        // SwerveModule module = getModule(0);
        // module0drive_statorcurrent.set(module.getDriveMotor().getStatorCurrent().getValueAsDouble());
        // module0drive_velocity.set(module.getDriveMotor().getVelocity().getValueAsDouble());

        // final Pose2d estimated_pose = 
        // m_odometry.resetPosition(new Rotation2d( m_pigeon2.getAngle()), m_modulePositions,    );
        // m_odometry.addVisionMeasurement(m_Vision., ModuleCount);
    }
}