package org.firstinspires.ftc.teamcode;


import static android.os.Build.ID;

/**
 * These are the constants used in PID
 */
public interface PID_Constants {
    double
            KP_TURN = 0.014,
            KI_TURN = 0.0000000000018,
            KD_TURN = 0.00095,
            ID = 1;
    double
            KP_STRAIGHT = 0.03,
            KI_STRAIGHT = 0,
            KD_STRAIGHT = 0;
    double
            KP_RANGE = 0.2;

    double tickForAndymark = 1120;
    double wheelDiameter = 4;
    double cmToInches = 2.54;
    double CLICKS_PER_CM = (tickForAndymark / (wheelDiameter * cmToInches)) / Math.PI;
}