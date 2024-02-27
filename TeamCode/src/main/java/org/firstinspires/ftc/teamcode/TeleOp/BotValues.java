package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.DetonateFR;
import org.firstinspires.ftc.teamcode.Autonomous.ExplosiveTachyonicParticle;
import org.firstinspires.ftc.teamcode.Autonomous.RDXBlast;

public class BotValues
{
    // Arm
    public final static double LEFT_ARM_HOME = 0.1;
    public final static double LEFT_ARM_OUTTAKE = 0.9;
    public final static double RIGHT_ARM_HOME = 0.9;
    public final static double RIGHT_ARM_OUTTAKE = 0.1;

    // Wrist
    public final static double LEFT_WRIST_HOME = 0.1;
    public final static double LEFT_WRIST_OUTTAKE_DOWN = 0.2;
    public final static double LEFT_WRIST_INTAKE = 0.5;
    public final static double LEFT_WRIST_OUTTAKE_UP = 0.9;
    public final static double RIGHT_WRIST_HOME = 0.1;
    public final static double RIGHT_WRIST_OUTTAKE_DOWN = 0.2;
    public final static double RIGHT_WRIST_INTAKE = 0.5;
    public final static double RIGHT_WRIST_OUTTAKE_UP = 0.9;

    // Claw
    public final static double LEFT_CLAW_HOME = 0.1;
    public final static double LEFT_CLAW_RANGE = 0.9;
    public final static double RIGHT_CLAW_HOME = 0.9;
    public final static double RIGHT_CLAW_RANGE = 0.1;

    // Plane Launcher
    public final static double PLANE_LAUNCHER_HOME = 0.1;
    public final static double PLANE_LAUNCHER_RANGE = 0.9;

    // Slides
    public final static double SLIDE_HUB_DIAMETER = 1.5; // in inches
    public final static double HIGH_SET_LINE = 20.1; // in inches
    public final static double MEDIUM_SET_LINE = 16; // in inches
    public final static double LOW_SET_LINE = 12.375; // in inches
    public static double slideUpAutoPow = voltageNormalize(0.8);
    public static double slideDownAutoPow = voltageNormalize(-0.5);
    public static double slideUpManualPow = voltageNormalize(0.4);
    public static double slideDownManualPow = voltageNormalize(-0.3);

    // Drivetrain
    public static double pow = voltageNormalize(0.77);
    public static double slowPow = voltageNormalize(0.2);
    public final static double driveStickDeadZone = 0.05;
    public final static double turningDeadZone = 0.05;
    public final static double QUICK_TURN_ANGLE = 45;

    // Hanging
    public static double hangPow = voltageNormalize(1);

    // DC Motor Encoder Resolutions
    public final static double TICKS_PER_REV_312 = 537.7;

    // Distance Sensor
    public final static double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches

    // Computer Vision
    public final static Float INFERENCE_CONFIDENCE_THRESHOLD = 0.5f;
    public final static int RESOLUTION_WIDTH = 1280;
    public final static int RESOLUTION_HEIGHT = 720;
    public final static String MODEL_NAME = "redBack.tflite";

    // Normalize Power By Voltage
    public static double voltageNormalize(double power) throws NullPointerException
    {
        if ((power * (12.0 / ExplosiveTachyon.voltageSensor.getVoltage())) > 1) {return 1.0;}
        else {return (power * (12.0 / ExplosiveTachyon.voltageSensor.getVoltage()));}
    }
    public static double voltageNormalizeForAuto(double power) throws NullPointerException
    {
        if ((power * (12.0 / ExplosiveTachyonicParticle.voltageSensor.getVoltage())) > 1) {return 1.0;}
        else {return (power * (12.0 / ExplosiveTachyonicParticle.voltageSensor.getVoltage()));}
    }

    // Automatic Acceleration / Deceleration
    public static double acceleratorTransform(double distance)
    {
        return Math.cos(distance * ((2 * Math.PI) / (4 * BACKDROP_SAFETY_DISTANCE)));
    }
}
