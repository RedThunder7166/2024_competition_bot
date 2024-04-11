package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum DynamicTag {
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

    SourceRight(9, 1),
    SourceLeft(10, 2),
    SubwooferShifted(3, 8),
    SubwooferCenter(4, 7),
    Amp(5, 6),
    StageRight(11, 15),
    StageLeft(12, 16),
    StageCenter(13, 14);

    public static Alliance alliance;

    private final int m_redID;
    private final int m_blueID;

    private int m_ID;

    private DynamicTag(int redID, int blueID) {
        m_redID = redID;
        m_blueID = blueID;
    }

    public int getID() {
        m_ID = alliance == Alliance.Red ? m_redID : m_blueID;
        return m_ID;
    }
}