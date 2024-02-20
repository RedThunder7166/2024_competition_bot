package frc.robot;

import frc.robot.Constants.LauncherConstants;

public final class Utils {
    public static double EncoderUnitsToAngle(double EncoderUnits) {
        return ((EncoderUnits * 360 )/ ((2048 * LauncherConstants.AIM_MOTOR_GEAR_RATIO * LauncherConstants.AIM_MOTOR_SPROCKET_RATIO)));
    }
}
