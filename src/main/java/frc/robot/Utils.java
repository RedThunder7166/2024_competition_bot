package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LauncherConstants;



public final class Utils {
    public static double EncoderUnitsToAngle(double EncoderUnits) {
        return ((EncoderUnits * 360 )/ ((2048 * LauncherConstants.AIM_MOTOR_GEAR_RATIO * LauncherConstants.AIM_MOTOR_SPROCKET_RATIO)));
    }

    /**
     * Divides second by 60
     *
     * @param second number of seconds
     * @return number of minutes
     */
    public static double secondToMinute(double second) {
        return second / 60;
    }
    /**
     * Mutiplies minute by 60
     *
     * @param minute number of minutes
     * @return number of seconds
     */
    public static double minuteToSecond(double minute) {
        return minute * 60;
    }

    // HIGH when not pressed, so negate
    public static boolean isBumperSwitchDown(DigitalInput bumperSwitch) {
        return !bumperSwitch.get();
    }

    public static boolean isAllenBradleyTripped(DigitalInput allenBradley) {
        return !allenBradley.get();
    }
}
