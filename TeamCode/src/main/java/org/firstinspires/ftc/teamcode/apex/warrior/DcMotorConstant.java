package org.firstinspires.ftc.teamcode.apex.warrior;

/**
 * This class is used to store constant values used for controlling DC Motors
 */
public class DcMotorConstant {

    //Constants used for ARM
    public static final int ARM_HIGH_POS = 450;
    public static final int ARM_LOW_POS = 30;

    //Constants used for LADDER
    public static final int LADDER_HIGH_POS = 1900;
    public static final int LADDER_LOW_POS = 0;
    public static final int LADDER_MID_POS = 850;

    //Constants used for HOOK
    public static final int HOOK_HIGH_POS = 1800;
    public static final int HOOK_LOW_POS = 40;

    // Adjust these for your robot
    public static final int MIN_TICKS = 0;       // encoder ticks = min angle
    public static final int MAX_TICKS = 2000;    // encoder ticks = max angle
    public static final double POWER = 0.75;     // motor power

}