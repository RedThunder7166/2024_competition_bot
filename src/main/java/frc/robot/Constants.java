package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
    public static final class AllianceAprilTagIDs {
        private static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        private static final boolean is_red_alliance = alliance == Alliance.Red;

        public static final int SOURCE_RIGHT = is_red_alliance ? 9 : 1;
        public static final int SOURCE_LEFT = is_red_alliance ? 10 : 2;

        public static final int SUBWOOFER_SHIFTED = is_red_alliance ? 3 : 8;
        public static final int SUBWOOFER_CENTER = is_red_alliance ? 4 : 7;

        public static final int AMP = is_red_alliance ? 5 : 6;

        // stupid official names that I cannot wrap my head around
        /* from https://www.youtube.com/watch?v=tElgzVLql08 (2024 Field Tour Video: Stage)
            .....................................................
            ...........................@.........................
            ...........................@.........................
            ...........................@.........................
            .........Stage.............@..........Stage..........
            .........Right.............@..........Left...........
            .........11/15.............@..........12/16..........
            ..........................@@@........................
            ........................@@@.@@@......................
            ......................@@@......@@@...................
            ....................@@@...........@@@................
            .................@@@.....Stage......@@@..............
            ...............@@@.......Center.......@@@............
            .............@@@.........13/14..........@@@..........
            .....................................................
        */
        public static final int STAGE_RIGHT = is_red_alliance ? 11 : 15;
        public static final int STAGE_LEFT = is_red_alliance ? 12 : 16; 
        public static final int STAGE_CENTER = is_red_alliance ? 13 : 14;
    }

    public static final class LauncherConstants {
        public static final int AIM_MOTOR_ID = 28;
        public static final double AIM_MOTOR_GEAR_RATIO = Math.pow(5, 3);

        public static final double AIM_MOTOR_HIGHEST_ANGLE_POSITIONS = -109;
        public static final double AIM_MOTOR_LOWEST_ANGLE_POSITIONS = -436;

        public static final int AIM_CANCODER_ID = -100; // TODO: CHANGE THIS
        public static final double AIM_CANCODER_LOADING_POSITION = 0; // TODO: CHANGE THIS
        public static final double ALLOWABLE_CANCODER_ERROR = 0.5;
    }
    public static final class ShooterConstants {
        public static final int TOP_MOTOR_ID = 10;
        public static final int BOTTOM_MOTOR_ID = 9;

        public static final double TARGET_VELOCITY_RPS = 80.0; // TODO: CHANGE THIS
    }
    public static final class IndexerConstants {
        public static final int MOTOR_ID = -100; // TODO: CHANGE THIS
        public static final int TARGET_VELOCITY = 80; // TODO: CHANGE THIS
    }
    public static final class ClimberConstants {
        public static final int LEFT_CLIMB_MOTOR_ID = -100; // TODO: CHANGE THIS
        public static final int RIGHT_CLIMB_MOTOR_ID = -100; // TODO: CHANGE THIS

        public static final int LEFT_ARM_LIMIT_SWITCH_ID = -100; // TODO: CHANGE THIS
        public static final int RIGHT_ARM_LIMIT_SWITCH_ID = -100; // TODO: CHANGE THIS
    }
    public static final class IntakeConstants {
        public static final int EXTENDER_MOTOR_ID = -100; // TODO: CHANGE THIS
        public static final int EXTENDER_CANCODER_ID = -100; // TODO: CHANGE THIS
        public static final int SUCKER_MOTOR_ID = -100; // TODO: CHANGE THIS

        public static final double EXTENDER_CANCODER_HOME_POSITION = 0.0; // TODO: CHANGE THIS
        public static final double EXTENDER_CANCODER_EXTENDED_POSITION = 0.0; // TODO: CHANGE THIS

        public static final int SUCKER_TARGET_VELOCITY_RPS = 80; // TODO: CHANGE THIS
    } 
}