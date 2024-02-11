package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ExplosiveTachyon extends LinearOpMode
{
    // Motors
    private DcMotorEx frontleft, backright, backleft, frontright;


    // Sensors
    private IMU imu;
    private Orientation lastAngles;
    private double currentAngle;
    private TouchSensor touchSensor;
    private ElapsedTime timer;
    private TouchSensor backdropSwitch;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor;


    // States
    private enum AutoDriveState {INITIAL, QUICK_CLOCKWISE, QUICK_COUNTERCLOCKWISE;}
    private AutoDriveState autoDriveState;

    // Controls
    private boolean slowForward, slowBackward, slowLeft, slowRight;
    private boolean counterClockwise, clockwise, quickClockwise, quickCounterClockwise, stopDrive;

    public void runOpMode()
    {
        initMotors();
        initSensors();
        initStates();
        telemetry.addData("C4", "Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            // Conditions for Controller1/Gamepad1
            slowForward = gamepad1.dpad_up;
            slowBackward = gamepad1.dpad_down;
            slowLeft = gamepad1.dpad_left;
            slowRight = gamepad1.dpad_right;
            counterClockwise = gamepad1.left_bumper;
            clockwise = gamepad1.right_bumper;
            stopDrive = gamepad1.left_trigger > 0.25;

            // Analog Driving
            double forward = -1 * gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double slope = (-1 * gamepad1.right_stick_y) / gamepad1.right_stick_x;
            double vertical = -1 * gamepad1.right_stick_y;
            double diagonal = (vertical / Math.abs(vertical)) * (Math.sqrt(Math.pow((gamepad1.right_stick_y), 2) + Math.pow((gamepad1.right_stick_x), 2)));

            double fLPower = forward + strafe;
            double fRPower = forward - strafe;
            double bLPower = forward - strafe;
            double bRPower = forward + strafe;

            if (slope > 0)
            {
                fLPower += diagonal;
                bRPower += diagonal;
            }
            else
            {
                fRPower += diagonal;
                bLPower += diagonal;
            }
            if (counterClockwise)
            {
                fLPower = -1 * BotValues.pow;
                fRPower = BotValues.pow;
                bLPower = -1 * BotValues.pow;
                bRPower = BotValues.pow;
            }
            else if (clockwise)
            {
                fLPower = BotValues.pow;
                fRPower = -1 * BotValues.pow;
                bLPower = BotValues.pow;
                bRPower = -1 * BotValues.pow;
            }
            if (stopDrive) {power(0);}
            else if (slowForward) {power(BotValues.slowPow);}
            else if (slowBackward) {power(-1 * BotValues.slowPow);}
            else if (slowLeft) {power(-1 * BotValues.slowPow, BotValues.slowPow, BotValues.slowPow, -1 * BotValues.slowPow);}
            else if (slowRight) {power(BotValues.slowPow, -1 * BotValues.slowPow, -1 * BotValues.slowPow, BotValues.slowPow);}
            else {power(Math.tanh(fLPower), Math.tanh(fRPower), Math.tanh(bLPower), Math.tanh(bRPower));}
        }
    }

    public void initMotors()
    {
        // Initialize Motors
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void initSensors()
    {
        lastAngles = new Orientation();
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        currentAngle = 0.0;
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        timer = new ElapsedTime();
        backdropSwitch = hardwareMap.get(TouchSensor.class, "backdrop switch");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance sensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance sensor");
    }

    public void initStates()
    {
        autoDriveState = AutoDriveState.INITIAL;
    }

    public void power(double pow)
    {
        frontleft.setPower(pow);
        frontright.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow);
    }

    public void power(double fL, double fR, double bL, double bR)
    {
        frontleft.setPower(fL);
        frontright.setPower(fR);
        backleft.setPower(bL);
        backright.setPower(bR);
    }
}
