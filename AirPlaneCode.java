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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//copy and paste is overpowered
@TeleOp (name="AirPlaneCode")
public class AirPlaneCode extends LinearOpMode {
    private DcMotorEx frontleft, backright, backleft, frontright;
    private DcMotorEx leftSlides, rightSlides;
    private Servo leftArm, rightArm;
    private Servo leftWrist, rightWrist;
    private Servo leftClaw, rightClaw;

    // Sensors
    private IMU imu;
    private Orientation lastAngles;
    private double currentAngle;
    private double startingAngle;
    private TouchSensor touchSensor;
    private ElapsedTime timer;
    private int slidePos;
    public static DistanceSensor distanceSensor;
    public static VoltageSensor voltageSensor;

    // States
    private enum SlideState {INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY};
    private AirPlaneCode.SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_INTAKE};
    private AirPlaneCode.ArmState armState, previousArmState;
    private enum WristState {FOLD, TO_INTAKE, INTAKE, TO_DOWN_OUTTAKE, DOWN_OUTTAKE, TO_UP_OUTTAKE, UP_OUTTAKE, TO_FOLD};
    private AirPlaneCode.WristState wristState, previousWristState;
    private enum ClawState {CLOSED, TO_OPEN, OPEN, TO_CLOSED};
    private AirPlaneCode.ClawState leftClawState, previousLeftClawState, rightClawState, previousRightClawState;

    // Controls
    private boolean forward, backward, left, right, slowForward, slowBackward, slowLeft, slowRight;
    private boolean counterClockwise, clockwise;
    private boolean stopDrive, slidesTouchingSensor;
    private boolean intakePOV, outtakePOV;
    private boolean slidesDown, slidesUpLow, slidesUpMid, slidesUpHigh, slidesUpManual, slidesDownManual, slideStop;
    private boolean armToIntake, armToOuttake;
    private boolean wristFold, wristToIntake, wristToDownOuttake, wristToUpOuttake;
    private boolean openLeftClaw, closeLeftClaw, openRightClaw, closeRightClaw;
    private static double driveStickY;

    // Flags
    private boolean slidePowered;

    @Override
    public void runOpMode()
    {
        initMotors();
        initSensors();
        initStates();
        telemetry.addData("C4", "Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !(isStopRequested()))
        {
            driveStickY = -1 * gamepad1.left_stick_y;
            updateControls();
            digitalDrive();
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

        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        leftSlides.setDirection(DcMotorEx.Direction.FORWARD);
        rightSlides.setDirection(DcMotorEx.Direction.REVERSE);

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
    }

    public void initSensors()
    {
        lastAngles = new Orientation();
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        currentAngle = 0.0;
        startingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //resetAngle();
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        timer = new ElapsedTime();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void initStates()
    {
        slideState = AirPlaneCode.SlideState.INITIAL;
        previousSlideState = slideState;
        slidePowered = false;
        armState = AirPlaneCode.ArmState.INTAKE;
        previousArmState = armState;
        wristState = AirPlaneCode.WristState.FOLD;
        previousWristState = wristState;
        leftClawState = AirPlaneCode.ClawState.CLOSED;
        previousLeftClawState = leftClawState;
        rightClawState = AirPlaneCode.ClawState.CLOSED;
        previousRightClawState = rightClawState;
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
        counterClockwise = gamepad1.left_trigger > BotValues.turningDeadZone;
        clockwise = gamepad1.right_trigger > BotValues.turningDeadZone;
        stopDrive = gamepad1.b;
        intakePOV = gamepad1.a;
        outtakePOV = gamepad1.y;

        // Conditions for Controller2/Gamepad2
        slidesDown = gamepad2.dpad_left;
        slidesUpHigh = gamepad2.dpad_up;
        slidesUpMid = gamepad2.dpad_right;
        slidesUpLow = gamepad2.dpad_down;
        slidesUpManual = gamepad2.left_stick_y < -0.25;
        slidesDownManual = gamepad2.left_stick_y > 0.25;
        slideStop = gamepad2.right_stick_button;
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
        else if (clockwise) {power(-1 * BotValues.pow, BotValues.pow, -1 * BotValues.pow, BotValues.pow);}
        else if (counterClockwise) {power(BotValues.pow, -1 * BotValues.pow, BotValues.pow, -1 * BotValues.pow);}

        // Stationary when not controlled
        else {power(0);}
    }

    public void analogDrive()
    {
        // Manual Acceleration/Deceleration using joystick and triggers

        if (stopDrive) {power(0);}
        else if (slowForward) {power(BotValues.slowPow);}
        else if (slowBackward) {power(-1 * BotValues.slowPow);}
        else if (slowLeft) {power(-1 * BotValues.slowPow, BotValues.slowPow, BotValues.slowPow, -1 * BotValues.slowPow);}
        else if (slowRight) {power(BotValues.slowPow, -1 * BotValues.slowPow, -1 * BotValues.slowPow, BotValues.slowPow);}
        else if (counterClockwise || clockwise)
        {
            // Turning
            double counterclockwise = gamepad1.left_trigger;
            double clockwise = gamepad1.right_trigger;

            if (counterclockwise < BotValues.turningDeadZone) {counterclockwise = 0;}
            if (clockwise < BotValues.turningDeadZone) {clockwise = 0;}

            double fLPower = BotValues.voltageNormalize(clockwise - counterclockwise);
            double fRPower = BotValues.voltageNormalize(counterclockwise - clockwise);
            double bLPower = BotValues.voltageNormalize(clockwise - counterclockwise);
            double bRPower = BotValues.voltageNormalize(counterclockwise - clockwise);

            power(fLPower, fRPower, bLPower, bRPower);
        }
        else
        {
            // Axial Movement
            double y = -1 * gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startingAngle;
            theta = BotValues.normalizeAngle(theta);
            double forward, strafe;

            if ((BotValues.angleRoundingPlace * theta) - (int)(BotValues.angleRoundingPlace * theta) >= 0.5)
            {theta = ((int)((BotValues.angleRoundingPlace * theta) + 1)) / BotValues.angleRoundingPlace;}
            else {theta = ((int)(BotValues.angleRoundingPlace * theta)) / BotValues.angleRoundingPlace;}

            if (Math.abs(y) < BotValues.driveStickDeadZoneLow || Math.abs(y) > BotValues.driveStickDeadZoneHigh) {forward = 0;}
            else {forward = (x * Math.sin(theta)) + (y * Math.cos(theta));}
            if (Math.abs(x) < BotValues.driveStickDeadZoneLow || Math.abs(x) > BotValues.driveStickDeadZoneHigh) {strafe = 0;}
            else {strafe = (x * Math.cos(theta)) - (y * Math.sin(theta));}

            double fLPower = BotValues.voltageNormalize(forward + strafe);
            double fRPower = BotValues.voltageNormalize(forward - strafe);
            double bLPower = BotValues.voltageNormalize(forward - strafe);
            double bRPower = BotValues.voltageNormalize(forward + strafe);

            power(fLPower, fRPower, bLPower, bRPower);
        }
    }

    public void triggerActions()
    {
        // Slides FSM
        if (slideStop)
        {
            leftSlides.setPower(0);
            rightSlides.setPower(0);
            slidePowered = false;
            previousSlideState = slideState;
            slideState = AirPlaneCode.SlideState.STATIONARY;
            telemetry.addData("Slides", "Stopped");
            telemetry.update();
        }
        else if (slideState == AirPlaneCode.SlideState.STATIONARY)
        {
            if (slidePowered)
            {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePowered = false;
            }
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.INITIAL)
        {
            if (slidePowered)
            {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePowered = false;
            }
            if (slidesUpLow && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_LOW;
            }
            else if (slidesUpMid && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MEDIUM;
            }
            else if (slidesUpHigh && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_HIGH;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.UP_LOW)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.LOW;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == AirPlaneCode.SlideState.INITIAL) {distance = BotValues.LOW_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.MEDIUM) {distance = BotValues.LOW_SET_LINE - BotValues.MEDIUM_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.HIGH) {distance = BotValues.LOW_SET_LINE - BotValues.HIGH_SET_LINE;}
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
                    slidePowered = true;
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                    slidePowered = true;
                }
            }
        }
        else if (slideState == AirPlaneCode.SlideState.LOW)
        {
            if (slidePowered)
            {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePowered = false;
            }
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesUpMid && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MEDIUM;
            }
            else if (slidesUpHigh && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_HIGH;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.UP_MEDIUM)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.MEDIUM;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == AirPlaneCode.SlideState.INITIAL) {distance = BotValues.MEDIUM_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.LOW) {distance = BotValues.MEDIUM_SET_LINE - BotValues.LOW_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.HIGH) {distance = BotValues.MEDIUM_SET_LINE - BotValues.HIGH_SET_LINE;}
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
                    slidePowered = true;
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                    slidePowered = true;
                }
            }
        }
        else if (slideState == AirPlaneCode.SlideState.MEDIUM)
        {
            if (slidePowered)
            {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePowered = false;
            }
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesUpLow && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_LOW;
            }
            else if (slidesUpHigh && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_HIGH;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.UP_HIGH)
        {
            if (!(rightSlides.isBusy()) && (rightSlides.getCurrentPosition()) != 0)
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.HIGH;
            }
            else if (!(rightSlides.isBusy()))
            {
                double distance = 0;
                if (previousSlideState == AirPlaneCode.SlideState.INITIAL) {distance = BotValues.HIGH_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.LOW) {distance = BotValues.HIGH_SET_LINE - BotValues.LOW_SET_LINE;}
                else if (previousSlideState == AirPlaneCode.SlideState.MEDIUM) {distance = BotValues.HIGH_SET_LINE - BotValues.MEDIUM_SET_LINE;}
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
                    slidePowered = true;
                }
                else
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                    slidePowered = true;
                }
            }
        }
        else if (slideState == AirPlaneCode.SlideState.HIGH)
        {
            if (slidePowered)
            {
                leftSlides.setPower(0);
                rightSlides.setPower(0);
                rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePowered = false;
            }
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesUpLow && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_LOW;
            }
            else if (slidesUpMid && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MEDIUM;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN_MANUAL;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.DOWN)
        {
            if (slidesTouchingSensor)
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.INITIAL;
            }
            else
            {
                if (!(slidePowered))
                {
                    leftSlides.setPower(BotValues.slideDownAutoPow);
                    rightSlides.setPower(BotValues.slideDownAutoPow);
                    slidePowered = true;
                }
            }
        }
        else if (slideState == AirPlaneCode.SlideState.UP_MANUAL)
        {
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN_MANUAL;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                if (!(slidePowered))
                {
                    leftSlides.setPower(BotValues.slideUpManualPow);
                    rightSlides.setPower(BotValues.slideUpManualPow);
                    slidePowered = true;
                }
            }
            else
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.STATIONARY;
            }
        }
        else if (slideState == AirPlaneCode.SlideState.DOWN_MANUAL)
        {
            if (slidesDown && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.DOWN;
            }
            else if (slidesUpManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.UP_MANUAL;
            }
            else if (slidesDownManual && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                if (!(slidePowered))
                {
                    leftSlides.setPower(BotValues.slideDownManualPow);
                    rightSlides.setPower(BotValues.slideDownManualPow);
                    slidePowered = true;
                }
            }
            else
            {
                previousSlideState = slideState;
                slideState = AirPlaneCode.SlideState.STATIONARY;
            }
        }


        // Arm
        if (armState == AirPlaneCode.ArmState.INTAKE)
        {
            if (armToOuttake && (wristState == AirPlaneCode.WristState.INTAKE) && (leftClawState == AirPlaneCode.ClawState.CLOSED) && (rightClawState == AirPlaneCode.ClawState.CLOSED))
            {
                previousArmState = armState;
                armState = AirPlaneCode.ArmState.TO_OUTTAKE;
            }
        }
        else if (armState == AirPlaneCode.ArmState.TO_OUTTAKE)
        {
            leftArm.setPosition(BotValues.LEFT_ARM_OUTTAKE);
            rightArm.setPosition(BotValues.RIGHT_ARM_OUTTAKE);
            previousArmState = armState;
            armState = AirPlaneCode.ArmState.OUTTAKE;
        }
        else if (armState == AirPlaneCode.ArmState.OUTTAKE)
        {
            if (armToIntake && (wristState == AirPlaneCode.WristState.INTAKE) && (slideState == AirPlaneCode.SlideState.INITIAL) && (leftClawState == AirPlaneCode.ClawState.CLOSED) && (rightClawState == AirPlaneCode.ClawState.CLOSED))
            {
                previousArmState = armState;
                armState = AirPlaneCode.ArmState.TO_INTAKE;
            }
        }
        else if (armState == AirPlaneCode.ArmState.TO_INTAKE)
        {
            leftArm.setPosition(BotValues.LEFT_ARM_HOME);
            rightArm.setPosition(BotValues.RIGHT_ARM_HOME);
            previousArmState = armState;
            armState = AirPlaneCode.ArmState.INTAKE;
        }


        // Wrist
        if (wristState == AirPlaneCode.WristState.FOLD)
        {
            if (wristToIntake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_INTAKE;
            }
            else if (wristToDownOuttake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_DOWN_OUTTAKE;
            }
            else if (wristToUpOuttake && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_UP_OUTTAKE;
            }
        }
        else if (wristState == AirPlaneCode.WristState.TO_INTAKE)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_INTAKE);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_INTAKE);
            previousWristState = wristState;
            wristState = AirPlaneCode.WristState.INTAKE;
        }
        else if (wristState == AirPlaneCode.WristState.INTAKE)
        {
            if (wristFold)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_FOLD;
            }
            else if (wristToDownOuttake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_DOWN_OUTTAKE;
            }
            else if (wristToUpOuttake && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_UP_OUTTAKE;
            }
        }
        else if (wristState == AirPlaneCode.WristState.TO_DOWN_OUTTAKE)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_DOWN);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_DOWN);
            previousWristState = wristState;
            wristState = AirPlaneCode.WristState.DOWN_OUTTAKE;
        }
        else if (wristState == AirPlaneCode.WristState.DOWN_OUTTAKE)
        {
            if (wristFold)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_FOLD;
            }
            else if (wristToIntake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_INTAKE;
            }
            else if (wristToUpOuttake && (armState != AirPlaneCode.ArmState.INTAKE))
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_UP_OUTTAKE;
            }
        }
        else if (wristState == AirPlaneCode.WristState.TO_UP_OUTTAKE)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_UP);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_UP);
            previousWristState = wristState;
            wristState = AirPlaneCode.WristState.UP_OUTTAKE;
        }
        else if (wristState == AirPlaneCode.WristState.UP_OUTTAKE)
        {
            if (wristFold)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_FOLD;
            }
            else if (wristToIntake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_INTAKE;
            }
            else if (wristToDownOuttake)
            {
                previousWristState = wristState;
                wristState = AirPlaneCode.WristState.TO_DOWN_OUTTAKE;
            }
        }
        else if (wristState == AirPlaneCode.WristState.TO_FOLD)
        {
            leftWrist.setPosition(BotValues.LEFT_WRIST_HOME);
            rightWrist.setPosition(BotValues.RIGHT_WRIST_HOME);
            previousWristState = wristState;
            wristState = AirPlaneCode.WristState.FOLD;
        }


        // Left Claw
        if (leftClawState == AirPlaneCode.ClawState.CLOSED)
        {
            if (openLeftClaw)
            {
                previousLeftClawState = leftClawState;
                leftClawState = AirPlaneCode.ClawState.TO_OPEN;
            }
        }
        else if (leftClawState == AirPlaneCode.ClawState.TO_OPEN)
        {
            leftClaw.setPosition(BotValues.LEFT_CLAW_RANGE);
            previousLeftClawState = leftClawState;
            leftClawState = AirPlaneCode.ClawState.OPEN;
        }
        else if (leftClawState == AirPlaneCode.ClawState.OPEN)
        {
            if (closeLeftClaw)
            {
                previousLeftClawState = leftClawState;
                leftClawState = AirPlaneCode.ClawState.TO_CLOSED;
            }
        }
        else if (leftClawState == AirPlaneCode.ClawState.TO_CLOSED)
        {
            leftClaw.setPosition(BotValues.LEFT_CLAW_HOME);
            previousLeftClawState = leftClawState;
            leftClawState = AirPlaneCode.ClawState.CLOSED;
        }


        // Right Claw
        if (rightClawState == AirPlaneCode.ClawState.CLOSED)
        {
            if (openRightClaw)
            {
                previousRightClawState = rightClawState;
                rightClawState = AirPlaneCode.ClawState.TO_OPEN;
            }
        }
        else if (rightClawState == AirPlaneCode.ClawState.TO_OPEN)
        {
            rightClaw.setPosition(BotValues.RIGHT_CLAW_RANGE);
            previousRightClawState = rightClawState;
            rightClawState = AirPlaneCode.ClawState.OPEN;
        }
        else if (rightClawState == AirPlaneCode.ClawState.OPEN)
        {
            if (closeRightClaw)
            {
                previousRightClawState = rightClawState;
                rightClawState = AirPlaneCode.ClawState.TO_CLOSED;
            }
        }
        else if (rightClawState == AirPlaneCode.ClawState.TO_CLOSED)
        {
            rightClaw.setPosition(BotValues.RIGHT_CLAW_HOME);
            previousRightClawState = rightClawState;
            rightClawState = AirPlaneCode.ClawState.CLOSED;
        }
    }

    public static double getDriveStick() {return driveStickY;}
}

