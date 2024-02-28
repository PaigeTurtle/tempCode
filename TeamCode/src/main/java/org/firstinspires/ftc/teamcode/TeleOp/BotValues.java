package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    // IMU
    public final static double angleRoundingPlace = 100.0; // 100.0 for rounding to hundredths place

    // Distance Sensor
    public final static double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches

    // Computer Vision
    public final static Float INFERENCE_CONFIDENCE_THRESHOLD = 0.5f;
    public final static int RESOLUTION_WIDTH = 1280;
    public final static int RESOLUTION_HEIGHT = 720;
    public final static String MODEL_NAME = "redBack.tflite";
    public final static int LEFT = 0;
    public final static int CENTER = 1;
    public final static int RIGHT = 2;

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



    // Road Runner
    public final static double MAX_DRIVE_RPM = 312;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_DRIVE_RPM / 60 * TICKS_PER_REV_312));
    public final static boolean RUN_USING_ENCODER = false;


     // These are physical constants that can be determined from your robot (including the track
     // width; it will be tune empirically later although a rough estimate is important). Users are
     // free to chose whichever linear distance unit they would like so long as it is consistently
     // used. The default values were selected with inches in mind. Road runner uses radians for
     // angular distances although most angular parameters are wrapped in Math.toRadians() for
     // convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 1; // in


     // These are the feedforward parameters used to model the drive motor behavior. If you are using
     // the built-in velocity PID, these values are fine as is. However, if you do not have drive
     // motor encoders or have elected not to use them for velocity control, these values should be
     // empirically tuned.
    public static double kV = 1.0 / rpmToVelocity(MAX_DRIVE_RPM); // Volt-seconds per meter
    public static double kA = 0; // Volt-seconds^2 per meter
    public static double kStatic = 0; // Volts


     // These values are used to generate the trajectories for you robot. To ensure proper operation,
     // the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     // Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     // small and gradually increase them later after everything is working. All distance units are
     // inches.
    public static double MAX_VEL = 52.276; // inches per second
    public static double MAX_ACCEL = 52.276; // inches per second per second
    public static double MAX_ANG_VEL = Math.toRadians(180); // radians per second
    public static double MAX_ANG_ACCEL = Math.toRadians(180); // radians per second per second


     // Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double encoderTicksToInches(double ticks) {return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV_312;}

    public static double rpmToVelocity(double rpm) {return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;}

    public static double getMotorVelocityF(double ticksPerSecond)
    {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
