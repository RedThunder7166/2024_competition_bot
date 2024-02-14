package frc.robot;

import frc.robot.Constants.LauncherConstants;

public final class Utils {
    public static double angleToEncoderUnits(double angle) {
        return (angle * ((LauncherConstants.AIM_MOTOR_GEAR_RATIO * 2048) / 360));
    }
}
