package frc.robot;

import frc.robot.Constants.LauncherConstants;

import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

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
}
