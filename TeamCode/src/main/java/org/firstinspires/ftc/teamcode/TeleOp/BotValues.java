package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Autonomous.ExplosiveTachyonicParticle;

public class BotValues
{
    // Arm
    public final static double LEFT_ARM_HOME = 0.1;
    public final static double LEFT_ARM_OUTTAKE = 0.8;
    public final static double RIGHT_ARM_HOME = 0.7;
    public final static double RIGHT_ARM_OUTTAKE = 0;

    // Wrist
    public final static double LEFT_WRIST_HOME = 0.1;
    public final static double LEFT_WRIST_OUTTAKE_DOWN = 0.1;
    public final static double LEFT_WRIST_INTAKE = 0.35;
    public final static double LEFT_WRIST_OUTTAKE_UP = 0.65;
    public final static double RIGHT_WRIST_HOME = 0.9;
    public final static double RIGHT_WRIST_OUTTAKE_DOWN = 0.85;
    public final static double RIGHT_WRIST_INTAKE = 0.35;
    public final static double RIGHT_WRIST_OUTTAKE_UP = 0.35;

    // Claw
    public final static double LEFT_CLAW_HOME = 0.9;
    public final static double LEFT_CLAW_RANGE = 0.5;
    public final static double RIGHT_CLAW_HOME = 0.1;
    public final static double RIGHT_CLAW_RANGE = 0.5;

    // Plane Launcher
    public final static double PLANE_LAUNCHER_HOME = 0.1;
    public final static double PLANE_LAUNCHER_RANGE = 0.4;

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
    public final static double driveStickDeadZoneLow = 0.05;
    public final static double driveStickDeadZoneHigh = 0.9;
    public final static double turningDeadZone = 0.05;
    public final static double QUICK_TURN_ANGLE = Math.toRadians(45);

    // Hanging
    public static double hangPow = voltageNormalize(1);

    // DC Motor Encoder Resolutions
    public final static double TICKS_PER_REV_312 = 537.7;

    // IMU
    public final static double angleRoundingPlace = 100.0; // 100.0 for rounding to hundredths place
    public static double normalizeAngle(double radians)
    {
        double angle = radians;
        if (radians > Math.PI) {angle -= (2 * Math.PI);}
        else if (radians <= (-1 * Math.PI)) {angle += (2 * Math.PI);}
        return angle;
    }

    // Distance Sensor
    public final static double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches

    // Computer Vision
    public final static Float INFERENCE_CONFIDENCE_THRESHOLD = 0.5f;
    public final static int RESOLUTION_WIDTH = 1280;
    public final static int RESOLUTION_HEIGHT = 720;
    public final static String RB_MODEL_NAME = "redBack.tflite";
    public final static String RA_MODEL_NAME = "redFront.tflite";
    public final static String BB_MODEL_NAME = "blueBack.tflite";
    public final static String BA_MODEL_NAME = "blueFront.tflite";
    public final static int PROP_LEFT = 0;
    public final static int PROP_CENTER = 1;
    public final static int PROP_RIGHT = 2;
    public final static int BLUE_LEFT_APRIL_TAG = 1;
    public final static int BLUE_CENTER_APRIL_TAG = 2;
    public final static int BLUE_RIGHT_APRIL_TAG = 3;
    public final static int RED_LEFT_APRIL_TAG = 4;
    public final static int RED_CENTER_APRIL_TAG = 5;
    public final static int RED_RIGHT_APRIL_TAG = 6;
    public final static double CAMERA_DISTANCE_TO_CLAW = 3.0; // inches from camera to center of outtaking claw

    // Pixels


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

    /////////////////////////////// Robot Positions /////////////////////////////////////

    // Starting Position
    public static final Pose2d startPose = new Pose2d(66, -36, Math.toRadians(0));

    // Spike Marks
    public static final Vector2d leftSpikeRA = new Vector2d(30, -47);
    public static final Vector2d centerSpikeRA = new Vector2d(25, -36);
    public static final Vector2d rightSpikeRA = new Vector2d(30, -25);

    public static final Vector2d leftSpikeRB = new Vector2d(30, 1);
    public static final Vector2d centerSpikeRB = new Vector2d(25, 12);
    public static final Vector2d rightSpikeRB = new Vector2d(30, 23);

    public static final Vector2d leftSpikeBA = new Vector2d(-30, -25);
    public static final Vector2d centerSpikeBA = new Vector2d(-25, -30);
    public static final Vector2d rightSpikeBA = new Vector2d(-30, -47);

    public static final Vector2d leftSpikeBB = new Vector2d(-30, 23);
    public static final Vector2d centerSpikeBB = new Vector2d(-25, 12);
    public static final Vector2d rightSpikeBB = new Vector2d(-30, 1);

    // Stacks
    public static final Vector2d stack1 = new Vector2d(-36, -71);
    public static final Vector2d stack2 = new Vector2d(-24, -71);
    public static final Vector2d stack3 = new Vector2d(-12, -71);
    public static final Vector2d stack4 = new Vector2d(12, -71);
    public static final Vector2d stack5 = new Vector2d(24, -71);
    public static final Vector2d stack6 = new Vector2d(36, -71);

    // April Tag Detection Spots
    public static final Vector2d aprilTagDetectionSpotRed = new Vector2d(18, 36);
    public static final Vector2d aprilTagDetectionSpotBlue = new Vector2d(-18, 36);

    // Backdrop Spots for Scoring
    public static final Vector2d backdropRedLeft = new Vector2d(30, 52);
    public static final Vector2d backdropRedCenter = new Vector2d(36, 52);
    public static final Vector2d backdropRedRight = new Vector2d(42, 52);
    public static final Vector2d backdropBlueLeft = new Vector2d(-42, 52);
    public static final Vector2d backdropBlueCenter = new Vector2d(-36, 52);
    public static final Vector2d backdropBlueRight = new Vector2d(-30, 52);

    // Parking
    public static final Vector2d redParking1 = new Vector2d(12, 60);
    public static final Vector2d redParking2 = new Vector2d(60, 60);
    public static final Vector2d blueParking1 = new Vector2d(-12, 60);
    public static final Vector2d blueParking2 = new Vector2d(-60, 60);


    ///////////////////////////////// Road Runner Info ///////////////////////////////
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
