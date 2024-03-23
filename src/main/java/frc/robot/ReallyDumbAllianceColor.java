// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class ReallyDumbAllianceColor {
    private static Alliance alliance;
    
    public static void setAlliance(Alliance new_alliance) {
        alliance = new_alliance;
    }
    public static Alliance getAlliance() {
        return alliance;
    }
}
