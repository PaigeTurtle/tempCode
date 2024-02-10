package org.firstinspires.ftc.teamcode.TeleOp;

public class BotValues
{
    // Arm
    public final static double LEFT_ARM_HOME = 0;
    public final static double LEFT_ARM_OUTTAKE = 1;
    public final static double RIGHT_ARM_HOME = LEFT_ARM_OUTTAKE;
    public final static double RIGHT_ARM_OUTTAKE = LEFT_ARM_HOME;

    // Wrist
    public final static double LEFT_WRIST_HOME = 0;
    public final static double LEFT_WRIST_OUTTAKE = 1;
    public final static double RIGHT_WRIST_HOME = LEFT_WRIST_OUTTAKE;
    public final static double RIGHT_WRIST_OUTTAKE = LEFT_WRIST_HOME;

    // Claw
    public final static double LEFT_CLAW_HOME = 0;
    public final static double LEFT_CLAW_RANGE = 1;
    public final static double RIGHT_CLAW_HOME = LEFT_CLAW_RANGE;
    public final static double RIGHT_CLAW_RANGE = LEFT_CLAW_HOME;

    // Slides
    public final static double SLIDE_HUB_DIAMETER = 1.5; // in inches
    public final static double HIGH_SET_LINE = 20.1; // in inches
    public final static double MEDIUM_SET_LINE = 16; // in inches
    public final static double LOW_SET_LINE = 12.375; // in inches
    public final static double slideUpAutoPow = 0.8;
    public final static double slideDownAutoPow = -0.5;
    public final static double slideUpManualPow = 0.4;
    public final static double slideDownManualPow = -0.3;

    // Drivetrain
    public final static double pow = 0.77;
    public final static double slowPow = 0.2;
    public final static double QUICK_TURN_ANGLE = 45;

    // DC Motor Encoder Resolutions
    public final static double TICKS_PER_REV_312 = 537.7;
}
