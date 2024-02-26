package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ImageRecognition;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;

@Autonomous (name = "Detonate FR")
public class DetonateFR extends LinearOpMode
{
    // Computer Vision
    private VisionPortal portal;
    private CameraName camera;
    private ImageRecognition classifier;
    private AprilTagProcessor aprilTagProcessor;
    private boolean targetFound;
    private AprilTagDetection desiredTag;
    private List<Classifications> results;
    private List<Category> categories;
    private List<AprilTagDetection> currentDetections;
    private int desiredTagID;
    private int label, frameNum;
    private long inferenceTime;


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
    public static DistanceSensor distanceSensor;
    public static VoltageSensor voltageSensor;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;

    // States
    private enum AutoState {RED_BACKDROP, RED_AUDIENCE, BLUE_BACKDROP, BLUE_AUDIENCE};
    private AutoState autoState;
    private enum DriveState
    {
        INITIAL, TO_SPIKE_MARK, AT_SPIKE_MARK, TO_DETECTION_SPOT, LOOKING_FOR_APRIL_TAG, PREPARE_YELLOW_OUTTAKE, SCORE_YELLOW,
        TO_STACK_1, TO_STACK_2, TO_STACK_3, AT_STACK, TO_BACKDROP, AT_BACKDROP, PARKING, PARKED;
    }
    private DriveState driveState;
    // States
    private enum SlideState {INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY};
    private SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_INTAKE};
    private ArmState armState;
    private enum WristState {FOLD, TO_INTAKE, INTAKE, TO_DOWN_OUTTAKE, DOWN_OUTTAKE, TO_UP_OUTTAKE, UP_OUTTAKE, TO_FOLD};
    private WristState wristState;


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        initCV();
        initMotors();
        initSensors();
        finalizeAutoState();
        telemetry.addData("C4", "Ready");
        telemetry.update();

        // Recognize position of prop on spike marks during Init, takes the latest recognition
        while (opModeInInit())
        {
            recognizePosition();
            telemetry.addData("Recognition", label);
            telemetry.update();
        }
        desiredTagID = label;

        // Clear up memory
        classifier.clearImageClassifier();

        while (opModeIsActive())
        {
            if (autoState == AutoState.RED_AUDIENCE) {triggerActionsRedAudience();}
            else if (autoState == AutoState.BLUE_BACKDROP) {triggerActionsBlueBack();}
            else if (autoState == AutoState.BLUE_AUDIENCE) {triggerActionsBlueAudience();}
            else {triggerActionsRedBack();}
        }

        // Place Purple Pixel on Spike Mark
        if (label == 0)
        {
            // left spike mark

        }
        else if (label == 2)
        {
            // right

        }
        else
        {
            // center

        }


        // Act on prop recognition
        goToCorrectAprilTag();
    }

    // Computer Vision Methods
    public void initCV()
    {
        // Clear capture directory for new webcam pictures
        File dir = new File("/sdcard/VisionPortal-Capture");
        String[] children = dir.list();
        for (int i = 0; i < children.length; i++)
        {
            boolean deleted = new File(dir, children[i]).delete();
            telemetry.addData("File Delete", "" + deleted);
            telemetry.update();
        }

        // Initialize Computer Vision Stuff
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        portal = (new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(BotValues.RESOLUTION_WIDTH, BotValues.RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor)).build();
        portal.stopLiveView();
        classifier = new ImageRecognition(BotValues.INFERENCE_CONFIDENCE_THRESHOLD, 1, 3, 0, 0, BotValues.MODEL_NAME);
        frameNum = 0;
        label = -1;
        desiredTagID = -1;
        targetFound = false;
        desiredTag  = null;
        currentDetections = null;
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
        autoState = AutoState.RED_BACKDROP;
        driveState = DriveState.INITIAL;
        slideState = SlideState.INITIAL;
        armState = ArmState.INTAKE;
        wristState = WristState.FOLD;
    }

    public void finalizeAutoState()
    {
        telemetry.addData("Please Select Alliance", "B for Red, X for Blue");
        telemetry.addData("Please Select Side", "Up Arrow for Backdrop Side, Down Arrow for Audience Side");
        telemetry.update();

        while (!(gamepad1.b || gamepad1.x))
        {
            telemetry.addData("Please Select Alliance", "B for Red, X for Blue");
            telemetry.addData("Please Select Side", "Up Arrow for Backdrop Side, Down Arrow for Audience Side");
            telemetry.update();
        }
        while (!(gamepad1.dpad_up || gamepad1.dpad_down))
        {
            telemetry.addData("Please Select Alliance", "B for Red, X for Blue");
            telemetry.addData("Please Select Side", "Up Arrow for Backdrop Side, Down Arrow for Audience Side");
            telemetry.update();
        }

        if (gamepad1.b && gamepad1.dpad_down) {autoState = AutoState.RED_AUDIENCE;}
        else if (gamepad1.x && gamepad1.dpad_up) {autoState = AutoState.BLUE_BACKDROP;}
        else if (gamepad1.x && gamepad1.dpad_down) {autoState = AutoState.BLUE_AUDIENCE;}
        else if (gamepad1.b && gamepad1.dpad_up) {autoState = AutoState.RED_BACKDROP;}
        else
        {
            while (opModeInInit() || opModeIsActive())
            {
                telemetry.addData("Error", "Alliance and Side Not Selected Correctly");
                telemetry.addLine("Please stop and restart OpMode");
                telemetry.update();
            }
        }

        if (autoState == AutoState.RED_BACKDROP) {telemetry.addData("Red Alliance", "Backdrop Side");}
        else if (autoState == AutoState.RED_AUDIENCE) {telemetry.addData("Red Alliance", "Audience Side");}
        else if (autoState == AutoState.BLUE_AUDIENCE) {telemetry.addData("Blue Alliance", "Audience Side");}
        else if (autoState == AutoState.BLUE_BACKDROP) {telemetry.addData("Blue Alliance", "Backdrop Side");}
    }

    public void recognizePosition()
    {
        ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
        sleep(250);
        File input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
        sleep(250);
        Bitmap bitmap = classifier.PNG2BMP(input);
        sleep(250);
        results = classifier.classify(bitmap, 0);
        telemetry.addData("Results", results);
        telemetry.update();
        while (results == null)
        {
            telemetry.addData("Recognition", "Null");
            telemetry.update();
            frameNum++;
            ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
            sleep(250);
            input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
            sleep(250);
            bitmap = classifier.PNG2BMP(input);
            sleep(250);
            results = classifier.classify(bitmap, 0);
        }
        for (Classifications detection : results)
        {
            categories = detection.getCategories();
            Category correctDetection = null;
            for (Category x : categories)
            {
                if (correctDetection == null) {correctDetection = x;}
                else if (x.getScore() > correctDetection.getScore()) {correctDetection = x;}
            }
            if (correctDetection == null) {break;}
            inferenceTime = classifier.getInferenceTime();
            telemetry.addData("Recognition", correctDetection.getDisplayName());
            telemetry.addLine(correctDetection.getLabel() + ": " + correctDetection.getScore());
            telemetry.addLine("Inference Time: " + inferenceTime);
            label = Integer.parseInt(correctDetection.getLabel());
            telemetry.update();
        }
        frameNum++;
    }

    public void goToCorrectAprilTag()
    {
        currentDetections = aprilTagProcessor.getDetections();
        sleep(250);
        while (currentDetections == null || currentDetections.size() == 0)
        {
            currentDetections = aprilTagProcessor.getFreshDetections();
            sleep(250);
        }
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        for (AprilTagDetection detection : currentDetections)
        {
            if (targetFound) {break;}
            else if (detection.metadata != null && detection.id == desiredTagID)
            {
                targetFound = true;
                telemetry.addLine("ID: " + detection.id + ", " + detection.metadata.name);
                desiredTag = detection;
            }
            else
            {
                telemetry.addData("Unknown Label", detection.id);
                telemetry.update();
            }
        }
        telemetry.update();
        if (targetFound)
        {
            /*// Sets current mode to using encoders
            frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            double  rangeError = desiredTag.ftcPose.y;
            double  bearingError = desiredTag.ftcPose.bearing;
            while (rangeError > BACKDROP_SAFETY_DISTANCE)
            {
                if (bearingError > 15)
                {
                    // strafe left
                    power(-0.3, 0.3, 0.3, -0.3);
                }
                else if (bearingError < -15)
                {
                    // strafe right
                    power(0.3, -0.3, -0.3, 0.3);
                }
                else
                {
                    // forward
                    power(acceleratorTransform(rangeError));
                }
                targetFound = false;
                currentDetections = aprilTagProcessor.getDetections();
                sleep(250);
                while (currentDetections == null || currentDetections.size() == 0)
                {
                    currentDetections = aprilTagProcessor.getFreshDetections();
                    sleep(250);
                }
                for (AprilTagDetection detection : currentDetections)
                {
                    if (targetFound) {break;}
                    else if (detection.metadata != null && detection.id == desiredTagID)
                    {
                        targetFound = true;
                        desiredTag = detection;
                    }
                }
                rangeError = desiredTag.ftcPose.range;
                bearingError = desiredTag.ftcPose.bearing;
            }
            power(0);
            sleep(50);*/
            //right(desiredTag.ftcPose.x);
            //slidesUp(1);
            //boxOut();
            //forward(desiredTag.ftcPose.y - SampleMecanumDrive.BACKDROP_SAFETY_DISTANCE);
        }
    }


    public void triggerActionsRedBack()
    {
        // Drivetrain FSM
        if (driveState == DriveState.INITIAL)
        {
            if (label != -1) {driveState = DriveState.TO_SPIKE_MARK;}
        }
        else if (driveState == DriveState.TO_SPIKE_MARK)
        {

        }
        else if (driveState == DriveState.AT_SPIKE_MARK)
        {

        }
        else if (driveState == DriveState.TO_DETECTION_SPOT)
        {

        }
        else if (driveState == DriveState.LOOKING_FOR_APRIL_TAG)
        {

        }
        else if (driveState == DriveState.PREPARE_YELLOW_OUTTAKE)
        {

        }
        else if (driveState == DriveState.SCORE_YELLOW)
        {

        }
        else if (driveState == DriveState.TO_STACK_1)
        {

        }
        else if (driveState == DriveState.TO_STACK_2)
        {

        }
        else if (driveState == DriveState.TO_STACK_3)
        {

        }
        else if (driveState == DriveState.AT_STACK)
        {

        }
        else if (driveState == DriveState.TO_BACKDROP)
        {

        }
        else if (driveState == DriveState.AT_BACKDROP)
        {

        }
        else if (driveState == DriveState.PARKING)
        {

        }
        else if (driveState == DriveState.PARKED)
        {

        }
    }

    public void triggerActionsRedAudience()
    {

    }

    public void triggerActionsBlueBack()
    {

    }

    public void triggerActionsBlueAudience()
    {

    }
}