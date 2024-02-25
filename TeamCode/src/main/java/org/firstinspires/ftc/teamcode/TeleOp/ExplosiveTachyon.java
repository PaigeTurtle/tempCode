package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class ExplosiveTachyon extends LinearOpMode
{
    // Motors
    private DcMotorEx frontleft, backright, backleft, frontright;
    private DcMotorEx leftSlides, rightSlides;
    private DcMotorEx leftHanger, rightHanger;
    private Servo leftArm, rightArm;
    private Servo leftWrist, rightWrist;
    private Servo leftClaw, rightClaw;
    private Servo planeLauncher;


    // Sensors
    private IMU imu;
    private Orientation lastAngles;
    private double currentAngle;
    private TouchSensor touchSensor;
    private ElapsedTime timer;
    private int slidePos;
    public static DistanceSensor distanceSensor;
    public static VoltageSensor voltageSensor;


    // States
    private enum SlideState {INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY};
    private SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_INTAKE};
    private ArmState armState;
    private enum WristState {FOLD, TO_INTAKE, INTAKE, TO_DOWN_OUTTAKE, DOWN_OUTTAKE, TO_UP_OUTTAKE, UP_OUTTAKE, TO_FOLD};
    private WristState wristState;


    // Controls
    private boolean forward, backward, left, right, slowForward, slowBackward, slowLeft, slowRight;
    private boolean counterClockwise, clockwise;
    private boolean stopDrive, closeToBackdrop, slidesTouchingSensor;
    private boolean hangUp, hangDown;
    private boolean launchAirplane;
    private boolean intakePOV, outtakePOV;
    private boolean slidesDown, slidesUpLow, slidesUpMid, slidesUpHigh, slidesUpManual, slidesDownManual, slideStop;
    private boolean armToIntake, armToOuttake;
    private boolean wristFold, wristToIntake, wristToDownOuttake, wristToUpOuttake;
    private boolean openLeftClaw, closeLeftClaw, openRightClaw, closeRightClaw;

    @Override
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
            updateControls();
            analogDrive();
            triggerActions();
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

        leftSlides = hardwareMap.get(DcMotorEx.class, "left slides");
        leftSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftSlides.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        rightSlides = hardwareMap.get(DcMotorEx.class, "right slides");
        rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSlides.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        leftHanger = hardwareMap.get(DcMotorEx.class, "left hanger");
        leftHanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftHanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftHanger.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        rightHanger = hardwareMap.get(DcMotorEx.class, "right hanger");
        rightHanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightHanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHanger.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        leftSlides.setDirection(DcMotorEx.Direction.FORWARD);
        rightSlides.setDirection(DcMotorEx.Direction.REVERSE);
        leftHanger.setDirection(DcMotorEx.Direction.FORWARD);
        rightHanger.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize Servos
        leftArm = hardwareMap.get(Servo.class, "left arm");
        leftArm.setDirection(Servo.Direction.FORWARD);
        leftArm.setPosition(BotValues.LEFT_ARM_HOME);

        rightArm = hardwareMap.get(Servo.class, "right arm");
        rightArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setPosition(BotValues.RIGHT_ARM_HOME);

        leftWrist = hardwareMap.get(Servo.class, "left wrist");
        leftWrist.setDirection(Servo.Direction.FORWARD);
        leftWrist.setPosition(BotValues.LEFT_WRIST_HOME);

        rightWrist = hardwareMap.get(Servo.class, "right wrist");
        rightWrist.setDirection(Servo.Direction.FORWARD);
        rightWrist.setPosition(BotValues.RIGHT_WRIST_HOME);

        leftClaw = hardwareMap.get(Servo.class, "left claw");
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(BotValues.LEFT_CLAW_HOME);

        rightClaw = hardwareMap.get(Servo.class, "right claw");
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(BotValues.RIGHT_CLAW_HOME);

        planeLauncher = hardwareMap.get(Servo.class, "plane launcher");
        planeLauncher.setDirection(Servo.Direction.FORWARD);
        planeLauncher.setPosition(BotValues.PLANE_LAUNCHER_HOME);
    }

    public void initSensors()
    {
        lastAngles = new Orientation();
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        currentAngle = 0.0;
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        timer = new ElapsedTime();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void initStates()
    {
        slideState = SlideState.INITIAL;
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

    public void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }

    public double getAngle()
    {
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        double deltaTheta = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaTheta > 180) {deltaTheta -= 360;}
        else if (deltaTheta <= -180) {deltaTheta += 360;}
        currentAngle += deltaTheta;
        lastAngles = orientation;
        return currentAngle;
    }

    public void updateControls()
    {
        // Conditions for Controller1/Gamepad1
        forward = gamepad1.left_stick_y < -0.25;
        backward = gamepad1.left_stick_y > 0.25;
        left = gamepad1.left_bumper;
        right = gamepad1.right_bumper;
        slowForward = gamepad1.dpad_up;
        slowBackward = gamepad1.dpad_down;
        slowLeft = gamepad1.dpad_left;
        slowRight = gamepad1.dpad_right;
        counterClockwise = gamepad1.left_trigger > 0.25;
        clockwise = gamepad1.right_trigger > 0.25;
        stopDrive = gamepad1.b;
        closeToBackdrop = distanceSensor.getDistance(DistanceUnit.INCH) > BotValues.BACKDROP_SAFETY_DISTANCE;
        launchAirplane = gamepad1.x;
        hangUp = gamepad1.right_stick_y < -0.25;
        hangDown = gamepad1.right_stick_y > 0.25;
        intakePOV = gamepad1.a;
        outtakePOV = gamepad1.y;

        // Conditions for Controller2/Gamepad2
        slidesDown = gamepad2.dpad_left;
        slidesUpHigh = gamepad2.dpad_up;
        slidesUpMid = gamepad2.dpad_right;
        slidesUpLow = gamepad2.dpad_down;
        slidesUpManual = gamepad2.left_stick_y < -0.25;
        slidesDownManual = gamepad2.left_stick_y > 0.25;
        slideStop = gamepad2.left_stick_button;
        slidesTouchingSensor = touchSensor.isPressed();
        openLeftClaw = gamepad2.left_trigger > 0.25;
        closeLeftClaw = gamepad2.left_bumper;
        openRightClaw = gamepad2.right_trigger > 0.25;
        closeRightClaw = gamepad2.right_bumper;
        armToIntake = gamepad2.right_stick_y > 0.25;
        armToOuttake = gamepad2.right_stick_y < -0.25;
        wristFold = gamepad2.x;
        wristToIntake = gamepad2.b;
        wristToDownOuttake = gamepad2.a;
        wristToUpOuttake = gamepad2.y;
    }

    public void digitalDrive()
    {
        if (stopDrive) {power(0);} // Brake Regardless

        // Slow Diagonal
        else if (slowForward && left) {power(0, BotValues.slowPow, BotValues.slowPow, 0);}
        else if (slowForward && right) {power(BotValues.slowPow, 0, 0, BotValues.slowPow);}
        else if (slowBackward && left) {power(-1 * BotValues.slowPow, 0, 0, -1 * BotValues.slowPow);}
        else if (slowBackward && right) {power(0, -1 * BotValues.slowPow, -1 * BotValues.slowPow, 0);}

        // Fast Diagonal
        else if (forward && left) {power(0, BotValues.pow, BotValues.pow, 0);}
        else if (forward && right) {power(BotValues.pow, 0, 0, BotValues.pow);}
        else if (backward && left) {power(-1 * BotValues.pow, 0, 0, -1 * BotValues.pow);}
        else if (backward && right) {power(0, -1 * BotValues.pow, -1 * BotValues.pow, 0);}

        // Slow Axial Movement
        else if (slowForward) {power(BotValues.slowPow);}
        else if (slowBackward) {power(-1 * BotValues.slowPow);}
        else if (slowLeft) {power(-1 * BotValues.slowPow, BotValues.slowPow, BotValues.slowPow, -1 * BotValues.slowPow);}
        else if (slowRight) {power(BotValues.slowPow, -1 * BotValues.slowPow, -1 * BotValues.slowPow, BotValues.slowPow);}

        // Fast Axial Movement
        else if (forward) {power(BotValues.pow);}
        else if (backward) {power(-1 * BotValues.pow);}
        else if (left) {power(-1 * BotValues.pow, BotValues.pow, BotValues.pow, -1 * BotValues.pow);}
        else if (right) {power(BotValues.pow, -1 * BotValues.pow, -1 * BotValues.pow, BotValues.pow);}

        // Turning
        else if (counterClockwise) {power(-1 * BotValues.pow, BotValues.pow, -1 * BotValues.pow, BotValues.pow);}
        else if (clockwise) {power(BotValues.pow, -1 * BotValues.pow, BotValues.pow, -1 * BotValues.pow);}

        // Stationary when not controlled
        else {power(0);}
    }

    public void analogDrive()
    {
        // Manual Acceleration/Deceleration
        double forward = -1 * gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double slope = (-1 * gamepad1.right_stick_y) / (gamepad1.right_stick_x + 0.000001);
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
        if (stopDrive) {power(0);}
        else if (closeToBackdrop)
        {
            if (this.forward) {power(BotValues.slowPow);}
            else if (backward) {power(-1 * BotValues.slowPow);}
            else if (left) {power(-1 * BotValues.slowPow, BotValues.slowPow, BotValues.slowPow, -1 * BotValues.slowPow);}
            else if (right) {power(BotValues.slowPow, -1 * BotValues.slowPow, -1 * BotValues.slowPow, BotValues.slowPow);}
        }
        else if (slowForward) {power(BotValues.slowPow);}
        else if (slowBackward) {power(-1 * BotValues.slowPow);}
        else if (slowLeft) {power(-1 * BotValues.slowPow, BotValues.slowPow, BotValues.slowPow, -1 * BotValues.slowPow);}
        else if (slowRight) {power(BotValues.slowPow, -1 * BotValues.slowPow, -1 * BotValues.slowPow, BotValues.slowPow);}
        else {power(Math.tanh(fLPower), Math.tanh(fRPower), Math.tanh(bLPower), Math.tanh(bRPower));}
    }

    public void triggerActions()
    {
        // Slides FSM
        if (slideStop)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            slideState = SlideState.STATIONARY;
        }
        else if (slideState == SlideState.STATIONARY)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slidesDown) {slideState = SlideState.DOWN;}
            else if (slidesUpManual) {slideState = SlideState.UP_MANUAL;}
            else if (slidesDownManual) {slideState = SlideState.DOWN_MANUAL;}
        }
        else if (slideState == SlideState.INITIAL)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slidesUpLow)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_LOW;
            }
            else if (slidesUpMid)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MEDIUM;
            }
            else if (slidesUpHigh)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_HIGH;
            }
            else if (slidesUpManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MANUAL;
            }
        }
        else if (slideState == SlideState.UP_LOW)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                slideState = SlideState.LOW;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == SlideState.INITIAL) {distance = BotValues.LOW_SET_LINE;}
                else if (previousSlideState == SlideState.MEDIUM) {distance = BotValues.LOW_SET_LINE - BotValues.MEDIUM_SET_LINE;}
                else if (previousSlideState == SlideState.HIGH) {distance = BotValues.LOW_SET_LINE - BotValues.HIGH_SET_LINE;}
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePos = (int)((distance / (BotValues.SLIDE_HUB_DIAMETER * Math.PI)) * BotValues.TICKS_PER_REV_312);
                rightSlides.setTargetPosition(slidePos);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                // set power for moving up
                if (distance > 0)
                {
                    leftSlides.setPower(BotValues.slideUpAutoPow);
                    rightSlides.setPower(BotValues.slideUpAutoPow);
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                }
            }
        }
        else if (slideState == SlideState.LOW)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slidesDown)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN;
            }
            else if (slidesUpMid)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MEDIUM;
            }
            else if (slidesUpHigh)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_HIGH;
            }
            else if (slidesUpManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MANUAL;
            }
            else if (slidesDownManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == SlideState.UP_MEDIUM)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                slideState = SlideState.MEDIUM;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == SlideState.INITIAL) {distance = BotValues.MEDIUM_SET_LINE;}
                else if (previousSlideState == SlideState.LOW) {distance = BotValues.MEDIUM_SET_LINE - BotValues.LOW_SET_LINE;}
                else if (previousSlideState == SlideState.HIGH) {distance = BotValues.MEDIUM_SET_LINE - BotValues.HIGH_SET_LINE;}
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePos = (int)((distance / (BotValues.SLIDE_HUB_DIAMETER * Math.PI)) * BotValues.TICKS_PER_REV_312);
                rightSlides.setTargetPosition(slidePos);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                // set power for moving up
                if (distance > 0)
                {
                    leftSlides.setPower(BotValues.slideUpAutoPow);
                    rightSlides.setPower(BotValues.slideUpAutoPow);
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                }
            }
        }
        else if (slideState == SlideState.MEDIUM)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slidesDown)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN;
            }
            else if (slidesUpLow)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_LOW;
            }
            else if (slidesUpHigh)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_HIGH;
            }
            else if (slidesUpManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MANUAL;
            }
            else if (slidesDownManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == SlideState.UP_HIGH)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                slideState = SlideState.HIGH;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == SlideState.INITIAL) {distance = BotValues.HIGH_SET_LINE;}
                else if (previousSlideState == SlideState.LOW) {distance = BotValues.HIGH_SET_LINE - BotValues.LOW_SET_LINE;}
                else if (previousSlideState == SlideState.MEDIUM) {distance = BotValues.HIGH_SET_LINE - BotValues.MEDIUM_SET_LINE;}
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePos = (int)((distance / (BotValues.SLIDE_HUB_DIAMETER * Math.PI)) * BotValues.TICKS_PER_REV_312);
                rightSlides.setTargetPosition(slidePos);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                // set power for moving up
                if (distance > 0)
                {
                    leftSlides.setPower(BotValues.slideUpAutoPow);
                    rightSlides.setPower(BotValues.slideUpAutoPow);
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                }
            }
        }
        else if (slideState == SlideState.HIGH)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slidesDown)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN;
            }
            else if (slidesUpLow)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_LOW;
            }
            else if (slidesUpMid)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MEDIUM;
            }
            else if (slidesUpManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_MANUAL;
            }
            else if (slidesDownManual)
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == SlideState.DOWN)
        {
            if (slidesTouchingSensor) {slideState = SlideState.INITIAL;}
            else
            {
                leftSlides.setPower(BotValues.slideDownAutoPow);
                rightSlides.setPower(BotValues.slideDownAutoPow);
            }
        }
        else if (slideState == SlideState.UP_MANUAL)
        {
            if (!(slidesUpManual)) {slideState = SlideState.STATIONARY;}
            else if (slidesDown) {slideState = SlideState.DOWN;}
            else if (slidesDownManual) {slideState = SlideState.DOWN_MANUAL;}
            else
            {
                leftSlides.setPower(BotValues.slideUpManualPow);
                rightSlides.setPower(BotValues.slideUpManualPow);
            }
        }
        else if (slideState == SlideState.DOWN_MANUAL)
        {
            if (!(slidesDownManual)) {slideState = SlideState.STATIONARY;}
            else if (slidesDown) {slideState = SlideState.DOWN;}
            else if (slidesUpManual) {slideState = SlideState.UP_MANUAL;}
            else
            {
                leftSlides.setPower(BotValues.slideDownManualPow);
                rightSlides.setPower(BotValues.slideDownManualPow);
            }
        }

        // Arm
        if (armToIntake)
        {
            leftArm.setPosition(BotValues.LEFT_ARM_HOME);
            rightArm.setPosition(BotValues.RIGHT_ARM_HOME);
        }
        else if (armToOuttake)
        {
            leftArm.setPosition(BotValues.LEFT_ARM_OUTTAKE);
            rightArm.setPosition(BotValues.RIGHT_ARM_OUTTAKE);
        }

        // Wrist
        if (wristFold)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_HOME);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_HOME);
        }
        else if (wristToIntake)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_INTAKE);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_INTAKE);
        }
        else if (wristToDownOuttake)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_DOWN);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_DOWN);
        }
        else if (wristToUpOuttake)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_UP);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_UP);
        }


        // Claw
        if (openLeftClaw) {leftClaw.setPosition(BotValues.LEFT_CLAW_RANGE);}
        else if (closeLeftClaw) {leftClaw.setPosition(BotValues.LEFT_CLAW_HOME);}
        if (openRightClaw) {rightClaw.setPosition(BotValues.RIGHT_CLAW_RANGE);}
        else if (closeRightClaw) {rightClaw.setPosition(BotValues.RIGHT_CLAW_HOME);}


        // Plane Launcher
        if (launchAirplane) {planeLauncher.setPosition(BotValues.PLANE_LAUNCHER_RANGE);}


        // Hanging
        if (hangUp)
        {
            leftHanger.setPower(BotValues.hangPow);
            rightHanger.setPower(BotValues.hangPow);
        }
        else if (hangDown)
        {
            leftHanger.setPower(-1 * BotValues.hangPow);
            rightHanger.setPower(-1 * BotValues.hangPow);
        }
        else
        {
            leftHanger.setPower(0);
            rightHanger.setPower(0);
        }
    }
}
