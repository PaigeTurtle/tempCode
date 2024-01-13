package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ImageRecognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;

@Autonomous (name = "RDX Blast")
public class RDXBlast extends LinearOpMode
{
    // Constants
    private static final Float INFERENCE_CONFIDENCE_THRESHOLD = 0.5f;
    private final int RESOLUTION_WIDTH = 1280;
    private final int RESOLUTION_HEIGHT = 720;
    private final static String MODEL_NAME = "redBack.tflite";


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
    private DcMotorEx leftHanger, rightHanger;
    private DcMotorEx slides;
    private Servo leftArm, rightArm;
    private Servo leftClawTurner, rightClawTurner;
    private Servo leftClawOpener, rightClawOpener;
    private Servo leftHangerRelease, rightHangerRelease;
    private Servo planeLauncher, planeLock;


    // Sensors
    private IMU imu;
    private Orientation lastAngles;
    private double currentAngle;
    private TouchSensor touchSensor;
    private ElapsedTime timer;
    private TouchSensor backdropSwitch;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;

    // States
    private enum DriveState
    {
        INITIAL, TO_SPIKE_MARK, AT_SPIKE_MARK, TO_DETECTION_SPOT, AT_DETECTION_SPOT, TO_BACKDROP, AT_BACKDROP,
        TO_STACKS, AT_STACKS, PARKING, PARKED;
    }
    private DriveState driveState;
    private enum SlideState
    {
        INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY;
    }
    private SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_AIRPLANE, AIRPLANE, TO_INTAKE;}
    private ArmState armState;
    private enum TurnerState {INTAKE, TO_OUTTAKE_DOWN, OUTTAKE_DOWN, TO_OUTTAKE_UP, OUTTAKE_UP, TO_INTAKE;}
    private TurnerState turnerState;
    private enum ClawState {OPEN, TO_CLOSED, CLOSED, TO_OPEN;}
    private ClawState leftClawState, rightClawState;


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        initCV();
        initMotors();
        initSensors();
        telemetry.addData("C4", "Ready");
        telemetry.update();

        // Recognize position of prop on spike marks during Init, takes the latest recognition
        while (opModeInInit()) {recognizePosition();}
        desiredTagID = label;

        // Clear up memory
        classifier.clearImageClassifier();

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
        portal = (new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor)).build();
        portal.stopLiveView();
        classifier = new ImageRecognition(INFERENCE_CONFIDENCE_THRESHOLD, 1, 3, 0, 0, MODEL_NAME);
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

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftHanger = hardwareMap.get(DcMotorEx.class, "left hanger");
        leftHanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftHanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftHanger.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightHanger = hardwareMap.get(DcMotorEx.class, "right hanger");
        rightHanger.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightHanger.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHanger.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // May need to change this depending on how robot behaves
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        slides.setDirection(DcMotorEx.Direction.FORWARD);
        leftHanger.setDirection(DcMotorEx.Direction.REVERSE);
        rightHanger.setDirection(DcMotorEx.Direction.FORWARD);



        leftArm = hardwareMap.get(Servo.class, "left arm");
        leftArm.setDirection(Servo.Direction.REVERSE);
        leftArm.setPosition(SampleMecanumDrive.LEFT_ARM_HOME);

        rightArm = hardwareMap.get(Servo.class, "right arm");
        rightArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setPosition(SampleMecanumDrive.RIGHT_ARM_HOME);

        leftClawTurner = hardwareMap.get(Servo.class, "left claw turner");
        leftClawTurner.setDirection(Servo.Direction.REVERSE);
        leftClawTurner.setPosition(SampleMecanumDrive.LEFT_CLAW_TURNER_HOME);

        rightClawTurner = hardwareMap.get(Servo.class, "right claw turner");
        rightClawTurner.setDirection(Servo.Direction.FORWARD);
        rightClawTurner.setPosition(SampleMecanumDrive.RIGHT_CLAW_TURNER_HOME);

        leftClawOpener = hardwareMap.get(Servo.class, "left claw opener");
        leftClawOpener.setDirection(Servo.Direction.FORWARD);
        leftClawOpener.setPosition(SampleMecanumDrive.LEFT_CLAW_OPENER_HOME);

        rightClawOpener = hardwareMap.get(Servo.class, "right claw opener");
        rightClawOpener.setDirection(Servo.Direction.REVERSE);
        rightClawOpener.setPosition(SampleMecanumDrive.RIGHT_CLAW_OPENER_HOME);

        leftHangerRelease = hardwareMap.get(Servo.class, "left hanger release");
        leftHangerRelease.setDirection(Servo.Direction.FORWARD);
        leftHangerRelease.setPosition(SampleMecanumDrive.LEFT_HANGER_RELEASE_HOME);

        rightHangerRelease = hardwareMap.get(Servo.class, "right hanger release");
        rightHangerRelease.setDirection(Servo.Direction.REVERSE);
        rightHangerRelease.setPosition(SampleMecanumDrive.RIGHT_HANGER_RELEASE_HOME);

        planeLauncher = hardwareMap.get(Servo.class, "plane launcher");
        planeLauncher.setDirection(Servo.Direction.REVERSE);
        planeLauncher.setPosition(SampleMecanumDrive.PLANE_LAUNCHER_HOME);

        planeLock = hardwareMap.get(Servo.class, "plane lock");
        planeLock.setDirection(Servo.Direction.FORWARD);
        planeLock.setPosition(SampleMecanumDrive.PLANE_LOCK_HOME);
    }

    public void initSensors()
    {
        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        timer = new ElapsedTime();
        backdropSwitch = hardwareMap.get(TouchSensor.class, "backdrop switch");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance sensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance sensor");
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


    public void triggerActions()
    {
        // Drivetrain FSM

    }
}