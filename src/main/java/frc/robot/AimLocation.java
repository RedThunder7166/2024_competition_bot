// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class AimLocation {
    public final String name;
    public final double position;
    public final double shooter_speed;
    public final double feeder_speed;
    private AimLocation(String name_in, double position_in, double shooter_speed_in, double feeder_speed_in) {
        name = name_in;
        position = position_in;
        shooter_speed = shooter_speed_in;
        feeder_speed = feeder_speed_in;
    }

    public static final AimLocation Loading = new AimLocation("Loading",
        847.5,
        0.7, // this modified speaker speed
        0
    );
    public static final AimLocation Amp = new AimLocation("Amp",
        1018,
        0.12,
        0
    );
    public static final AimLocation Trap = new AimLocation("Trap",
        1050,
        0.8,
        0
    );
    public static final AimLocation Speaker = new AimLocation("Speaker",
        817,
        Loading.shooter_speed,
        0
    );
    public static final AimLocation Subwoofer = new AimLocation("Subwoofer",
        981,
        0.5,
        0
    );
    // 810 779

    // Loading(847.5),//.3767 // TODO: RECORD LOADING AIM CANCODER ABSOLUTEPOSITION
    // Amp(967),//.4301757
    // Trap(981),//0.43679151
    // Speaker(787);//.35

    // Amp(0.12, 0.3),
    // Trap(0.37, 0.3),
    // Speaker(0.7, 0.3);

    private static AimLocation location = Loading;
    public static AimLocation getAimLocation() {
        return location;
    }
    public static void setAimLocation(AimLocation new_location) {
        location = new_location;
    }
}
