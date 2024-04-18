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

    /*  measure distances, manually find 'working' angles,
     use table with logistic model to set angle automatically through desmos*/
    public static double distanceLaunchAngleCalculation(double distance) {
        // final double m = 3.2272* Math.pow(10, 13);
        // final double k = -1.43668;
        // final double x_zero = -18.0102;
        // final double b = 150.065;
        // return (distance * (m / (1 + Math.pow(Math.E, -k * (distance - x_zero)))) + b) + 2;

        final double m = -2296.26;
        final double h = -57.3682;
        final double k = 0.225084;
        final double x_zero = 17.45;
        final double b = 96.2790;

        return (m / (h + Math.pow(Math.E, -k * (distance - x_zero)))) + b;
      }
}
