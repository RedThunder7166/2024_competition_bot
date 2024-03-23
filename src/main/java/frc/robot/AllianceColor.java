// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// /** Add your docs here. */
// public final class AllianceColor {
//     public static final AllianceColor allianceColor = new AllianceColor();
//     public Alliance alliance;
//     public boolean is_red_alliance;

//     public int SOURCE_RIGHT;
//     public int SOURCE_LEFT;

//     public int SUBWOOFER_SHIFTED;
//     public int SUBWOOFER_CENTER;

//     public int AMP;

//     // stupid official names that I cannot wrap my head around
//     /* from https://www.youtube.com/watch?v=tElgzVLql08 (2024 Field Tour Video: Stage)
//         .....................................................
//         ...........................@.........................
//         ...........................@.........................
//         ...........................@.........................
//         .........Stage.............@..........Stage..........
//         .........Right.............@..........Left...........
//         .........11/15.............@..........12/16..........
//         ..........................@@@........................
//         ........................@@@.@@@......................
//         ......................@@@......@@@...................
//         ....................@@@...........@@@................
//         .................@@@.....Stage......@@@..............
//         ...............@@@.......Center.......@@@............
//         .............@@@.........13/14..........@@@..........
//         .....................................................
//     */
//     public int STAGE_RIGHT;
//     public int STAGE_LEFT;
//     public int STAGE_CENTER;

//     private AllianceColor() {
//         update(DriverStation.getAlliance().orElse(Alliance.Blue));
//     }
//     public void update(Alliance new_alliance) {
//         alliance = new_alliance;
//         is_red_alliance = alliance == Alliance.Red;
//         SOURCE_RIGHT = is_red_alliance ? 9 : 1;
//         SOURCE_LEFT = is_red_alliance ? 10 : 2;
//         SUBWOOFER_SHIFTED = is_red_alliance ? 3 : 8;
//         SUBWOOFER_CENTER = is_red_alliance ? 4 : 7;
//         AMP = is_red_alliance ? 5 : 6;
//         STAGE_RIGHT = is_red_alliance ? 11 : 15;
//         STAGE_LEFT = is_red_alliance ? 12 : 16; 
//         STAGE_CENTER = is_red_alliance ? 13 : 14;
//     }
// }

public final class AllianceColor {
    public static final int RED_SOURCE_RIGHT = 9;
    public static final int RED_SOURCE_LEFT = 10;
    public static final int RED_SUBWOOFER_SHIFTED = 3;
    public static final int RED_SUBWOOFER_CENTER = 4;
    public static final int RED_AMP = 5;
    public static final int RED_STAGE_RIGHT = 11;
    public static final int RED_STAGE_LEFT = 12; 
    public static final int RED_STAGE_CENTER = 13;

    public static final int BLUE_SOURCE_RIGHT = 1;
    public static final int BLUE_SOURCE_LEFT = 2;
    public static final int BLUE_SUBWOOFER_SHIFTED = 8;
    public static final int BLUE_SUBWOOFER_CENTER = 7;
    public static final int BLUE_AMP = 6;
    public static final int BLUE_STAGE_RIGHT = 15;
    public static final int BLUE_STAGE_LEFT = 16; 
    public static final int BLUE_STAGE_CENTER = 14;
}