package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
    public static final NetworkTable IN_MATCH_INFORMATION_TAB = NetworkTableInstance.getDefault().getTable("InMatchInformation");

    public static final class LauncherConstants {
        public static final int AIM_MOTOR_ID = 12;
        public static final double AIM_MOTOR_GEAR_RATIO = 125;
        public static final double AIM_MOTOR_SPROCKET_RATIO = 18;

        public static final double MANUAL_AIM_DEADBAND = 0.25;

        public static final double AIM_MOTOR_HIGHEST_POSITION = 215;
        public static final double AIM_MOTOR_LOWEST_POSITION = 148;

        public static final int AIM_CANCODER_ID = 26;
        public static final double ALLOWABLE_CANCODER_ERROR = 0.5;
    }
    public static final class ShooterConstants {
        public static final int TOP_MOTOR_ID = 10;
        public static final int BOTTOM_MOTOR_ID = 9;
        public static final int FEEDER_MOTOR_ID = 11;

        public static final double AIM_TO_LOADING_DELAY_SECONDS = 0.5;
        public static final double AMP_TO_LOADING_DELAY_SECONDS = 1;


        public static final int Indexer_Sensor_ID = 2;
        public static final int WHEEL_EXIT_SENSOR_ID = 1;
        public static final int FEEDER_SENSOR_ID = 0;

        public static final double TARGET_SHOOTER_RPS = 30.0;
        public static final double SHOOTER_UP_TO_SPEED_THRESHOLD = 78.0; // was 40

        public static final double TARGET_FEEDER_RPS_BACKWARDS = 6; // TODO: CHANGE THIS
        public static final double TARGET_FEEDER_RPS_SLOW_BACKWARDS = 2; // TODO: CHANGE THIS

        public static final double TARGET_FEEDER_RPS = 14; // TODO: CHANGE THIS
    }
    public static final class IndexerConstants {
        public static final int MOTOR_ID = 17;
        // public static final int TARGET_VELOCITY = 80; 

        // public static final int SENSOR_ID = 2;
    }
    public static final class ClimberConstants {
        public static final int LEFT_CLIMB_MOTOR_ID = 7;
        public static final int RIGHT_CLIMB_MOTOR_ID = 15;

        public static final double MANUAL_DEADBAND = 0.25;

        public static final double LEFT_TOP_POSITION = 274;
        public static final double RIGHT_TOP_POSITION = 278;
    }
    public static final class IntakeConstants {
        public static final int LOWER_MOTOR_ID = 13;
        public static final int UPPER_MOTOR_ID = 14;

        public static final int ENTRANCE_SENSOR_ID = 3;
    }

    public static final class LEDConstants {
        public static final int CANDLE_ID = 20;

        public static final int START_INDEX = 0;
        public static final int LED_COUNT = 47;

        public static final int COUNT_PER_GROUP = (int) Math.floor(LED_COUNT / 3);

        public static final int GROUP_1_START = 0;
        private static final int GROUP_1_END = COUNT_PER_GROUP;

        public static final int GROUP_2_START = GROUP_1_END + 1;
        private static final int GROUP_2_END = GROUP_2_START + GROUP_1_END;

        public static final int GROUP_3_START = GROUP_2_END + 1;
        private static final int GROUP_3_END = LED_COUNT;
    }

    public static final class DeflectorConstants {
        public static final int MOTOR_ID = 18;

        public static final double TARGET_AMPERES = 15;
        public static final double MAX_ABS_DUTY_CYCLE = 0.087;
    }

    public static final class VisionConstants {
        public static final double TARGET_SPEAKER_DISTANCE_METERS = 3.4;
        public static final double DISTANCE_BETWEEN_CAMERA_AND_FRONT_OF_ROBOT = 27.5;
    }

    public static final class JetEngineConstants {
        public static final int MOTOR_ID = 27;
    }
}
