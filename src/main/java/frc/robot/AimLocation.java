// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class AimLocation {
    public final String name;
    public final boolean auto_target;
    public final double position;
    public final double shooter_speed_rps;
    public final double feeder_speed;
    private AimLocation(String name_in, double position_in, double shooter_speed_rps_in, double feeder_speed_in) {
        name = name_in;
        position = position_in;
        auto_target = false;
        shooter_speed_rps = shooter_speed_rps_in;
        feeder_speed = feeder_speed_in;
    }
        private AimLocation(String name_in, boolean auto_target_in, double shooter_speed_rps_in, double feeder_speed_in) {
        name = name_in;
        position = 0;
        auto_target = auto_target_in;
        shooter_speed_rps = shooter_speed_rps_in;
        feeder_speed = feeder_speed_in;
    }

    public static final AimLocation Loading = new AimLocation("Loading",
        163.56, // 847.5
        80, // this modified speaker speed
        0
    );
    public static final AimLocation AutoTarget = new AimLocation("AutoTarget",
        true,
        Loading.shooter_speed_rps,//TODO Bring back to .7 once the shooter wheels are moved back
        0
    );
    public static final double amp_percent = 0.13;
    public static final AimLocation Amp = new AimLocation("Amp",
        200.2, // 203.6
        0, // 12
        0
    );
    public static final AimLocation Trap = new AimLocation("Trap",
        205, // 1050
        25,
        0
    );
    public static final AimLocation Speaker = new AimLocation("Speaker",
        153.4, // 817
        Loading.shooter_speed_rps,
        0
    );
    public static final AimLocation Subwoofer = new AimLocation("Subwoofer",
        196.2, // 981
        50,
        0
    );

    private static AimLocation location = Loading;
    public static AimLocation getAimLocation() {
        return location;
    }
    public static void setAimLocation(AimLocation new_location) {
        location = new_location;
    }
}
