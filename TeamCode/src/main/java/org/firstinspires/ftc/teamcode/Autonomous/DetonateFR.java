package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.util.Size;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;
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
    private VisionPortal propDetectionPortal;
    private VisionPortal aprilTagDetectionPortal;
    private CameraName intakeCamera;
    private CameraName outtakeCamera;
    private ImageRecognition classifier;
    private String modelName;
    private AprilTagProcessor aprilTagProcessor;
    private List<Classifications> results;
    private List<Category> categories;
    private List<AprilTagDetection> currentDetections;
    private AprilTagDetection chosenDetection;
    private int currentAprilTagIndex;
    private int desiredAprilTagID;
    private int label, frameNum;
    private long inferenceTime;


    // Robot
    private ExplosiveTachyonicParticle c4;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private Pose2d currentPose;
    private int slidePos;
    private double horizontalErrorToAprilTag, verticalErrorToAprilTag;

    // Timers
    private ElapsedTime armTimer, wristTimer, leftClawTimer, rightClawTimer, purplePixelTimer, yellowPixelTimer, driveTimer;
    private ElapsedTime aprilTagTimer, loopTimer;


    // Flags
    private boolean drivePowered, slidesPowered, armPowered, wristPowered, leftClawPowered, rightClawPowered;
    private boolean aprilTagDetectionsRequested, detectionSelected;


    // Trajectories
    Trajectory toSpikeMark, awayFromSpikeMark, toAprilTagDetectionSpot, adjustHorizontally, adjustVertically;
    Trajectory awayFromBackdrop, toParking;


    // States
    private enum AutoState {RED_BACKDROP, RED_AUDIENCE, BLUE_BACKDROP, BLUE_AUDIENCE}
    private AutoState autoState;
    private enum DriveState
    {
        INITIAL, TO_SPIKE_MARK, AT_SPIKE_MARK, AWAY_FROM_SPIKE_MARK, TO_DETECTION_SPOT, LOOKING_FOR_APRIL_TAG,
        ADJUSTING_HORIZONTALLY_APRIL_TAG, ADJUSTING_VERTICALLY_APRIL_TAG, SCORE_YELLOW, AWAY_FROM_BACKDROP, PARKING, PARKED
    }
    private DriveState driveState, previousDriveState;
    // States
    private enum SlideState {INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY}
    private SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_INTAKE}
    private ArmState armState, previousArmState;
    private enum WristState {FOLD, TO_INTAKE, INTAKE, TO_DOWN_OUTTAKE, DOWN_OUTTAKE, TO_UP_OUTTAKE, UP_OUTTAKE, TO_FOLD}
    private WristState wristState, previousWristState;
    private enum ClawState {CLOSED, TO_OPEN, OPEN, TO_CLOSED}
    private ClawState leftClawState, previousLeftClawState, rightClawState, previousRightClawState;
    private enum PixelState
    {
        NO_PIXELS, LEFT_1, RIGHT_1, LEFT_AND_RIGHT, LEFT_2, RIGHT_2, YELLOW_AND_WHITE, PURPLE_ONLY, YELLOW_ONLY, YELLOW_AND_PURPLE,
        DROPPING_PURPLE, DROPPING_YELLOW
    }
    private PixelState pixelState, previousPixelState;
    private enum AprilTagState {INITIAL, GETTING_DETECTIONS, VERIFYING_DETECTIONS, PROCESSING_DETECTIONS,
        REFRESHING_DETECTIONS, TARGET_FOUND}
    private AprilTagState aprilTagState, previousAprilTagState;


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        c4 = new ExplosiveTachyonicParticle(hardwareMap);
        finalizeAutoState();
        initCV();
        initStates();

        telemetry.addData("C4", "Ready");
        telemetry.update();

        // Recognize position of prop on spike marks during Init, takes the latest recognition
        while (opModeInInit())
        {
            recognizePosition();
            telemetry.addData("Recognition", label);
            telemetry.update();
        }
        if (autoState == AutoState.BLUE_AUDIENCE || autoState == AutoState.BLUE_BACKDROP) {desiredAprilTagID = label + 1;}
        else {desiredAprilTagID = label + 4;}

        classifier.clearImageClassifier(); // Clear up memory
        initDriveSequences();

        loopTimer.reset();
        while (opModeIsActive() && !(isStopRequested()))
        {
            triggerActionsSimple();
            telemetry.addData("Loop Time", loopTimer.milliseconds());
            telemetry.update();
            loopTimer.reset();
        }
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
        intakeCamera = hardwareMap.get(WebcamName.class, "intake camera");
        outtakeCamera = hardwareMap.get(WebcamName.class, "outtake camera");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        propDetectionPortal = (new VisionPortal.Builder()
                .setCamera(intakeCamera)
                .setCameraResolution(new Size(BotValues.INTAKE_RESOLUTION_WIDTH, BotValues.INTAKE_RESOLUTION_HEIGHT)))
                .build();
        propDetectionPortal.stopLiveView();
        aprilTagDetectionPortal = (new VisionPortal.Builder()
                .setCamera(outtakeCamera)
                .setCameraResolution(new Size(BotValues.OUTTAKE_RESOLUTION_WIDTH, BotValues.OUTTAKE_RESOLUTION_HEIGHT))
                .addProcessor(aprilTagProcessor))
                .build();
        aprilTagDetectionPortal.stopLiveView();
        if (autoState == AutoState.RED_AUDIENCE) {modelName = BotValues.RA_MODEL_NAME;}
        else if (autoState == AutoState.BLUE_BACKDROP) {modelName = BotValues.BB_MODEL_NAME;}
        else if (autoState == AutoState.BLUE_AUDIENCE) {modelName = BotValues.BA_MODEL_NAME;}
        else {modelName = BotValues.RB_MODEL_NAME;}
        classifier = new ImageRecognition(BotValues.INFERENCE_CONFIDENCE_THRESHOLD, 1, 3, 0, 0, modelName);
        frameNum = 0;
        label = -1;
        desiredAprilTagID = -1;
        currentDetections = null;
        chosenDetection = null;
        detectionSelected = false;
        currentAprilTagIndex = -1;
    }

    public void initStates()
    {
        autoState = AutoState.RED_BACKDROP;
        driveState = DriveState.INITIAL;
        previousDriveState = driveState;
        slideState = SlideState.INITIAL;
        previousSlideState = slideState;
        armState = ArmState.INTAKE;
        previousArmState = armState;
        wristState = WristState.FOLD;
        previousWristState = wristState;
        leftClawState = ClawState.CLOSED;
        previousLeftClawState = leftClawState;
        rightClawState = ClawState.CLOSED;
        previousRightClawState = rightClawState;
        pixelState = PixelState.YELLOW_AND_PURPLE;
        previousPixelState = pixelState;
        aprilTagState = AprilTagState.INITIAL;
        previousAprilTagState = aprilTagState;
        drivePowered = false;
        slidesPowered = false;
        armPowered = false;
        wristPowered = false;
        leftClawPowered = false;
        rightClawPowered = false;
        aprilTagDetectionsRequested = false;
        armTimer = new ElapsedTime();
        wristTimer = new ElapsedTime();
        leftClawTimer = new ElapsedTime();
        rightClawTimer = new ElapsedTime();
        purplePixelTimer = new ElapsedTime();
        yellowPixelTimer = new ElapsedTime();
        driveTimer = new ElapsedTime();
        aprilTagTimer = new ElapsedTime();
        loopTimer = new ElapsedTime();
    }

    public void initDriveSequences()
    {
        if (autoState == AutoState.RED_AUDIENCE)
        {

        }
        else if (autoState == AutoState.BLUE_BACKDROP)
        {
            currentPose = BotValues.startPoseBB;
            c4.setPoseEstimate(currentPose);
            if (label == BotValues.PROP_LEFT)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.leftSpikeBB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropBlueLeft, Math.toRadians(90))
                        .build();
            }
            else if (label == BotValues.PROP_CENTER)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.centerSpikeBB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropBlueCenter, Math.toRadians(90))
                        .build();
            }
            else if (label == BotValues.PROP_RIGHT)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(-48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.rightSpikeBB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropBlueRight, Math.toRadians(90))
                        .build();
            }
        }
        else if (autoState == AutoState.BLUE_AUDIENCE)
        {

        }
        else
        {
            currentPose = BotValues.startPoseRB;
            c4.setPoseEstimate(currentPose);
            if (label == BotValues.PROP_LEFT)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.leftSpikeRB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropRedLeft, Math.toRadians(90))
                        .build();
            }
            else if (label == BotValues.PROP_CENTER)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.centerSpikeRB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropRedCenter, Math.toRadians(90))
                        .build();
            }
            else if (label == BotValues.PROP_RIGHT)
            {
                toSpikeMark = c4.trajectoryBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(BotValues.rightSpikeRB, Math.toRadians(0))
                        .build();
                awayFromSpikeMark = c4.trajectoryBuilder(toSpikeMark.end())
                        .forward(BotValues.spikeMarkBackOutDistance)
                        .build();
                toAprilTagDetectionSpot = c4.trajectoryBuilder(awayFromSpikeMark.end())
                        .splineToConstantHeading(BotValues.backdropRedRight, Math.toRadians(90))
                        .build();
            }
        }
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
            while ((opModeInInit() || opModeIsActive()) && !(isStopRequested()))
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
        ((VisionPortalImpl)propDetectionPortal).saveNextFrameRaw("Capture/" + frameNum);
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
            ((VisionPortalImpl)propDetectionPortal).saveNextFrameRaw("Capture/" + frameNum);
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


    // Simpler ones
    public void triggerActionsSimple()
    {
        // Drivetrain FSM
        if (driveState == DriveState.INITIAL)
        {
            if (opModeIsActive())
            {
                previousDriveState = driveState;
                driveState = DriveState.TO_SPIKE_MARK;
            }
        }
        else if (driveState == DriveState.TO_SPIKE_MARK)
        {
            if (!(drivePowered))
            {
                c4.followTrajectoryAsync(toSpikeMark);
                drivePowered = true;
            }
            else if (!(c4.isBusy()))
            {
                previousDriveState = driveState;
                driveState = DriveState.AT_SPIKE_MARK;
                currentPose = toSpikeMark.end();
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.AT_SPIKE_MARK)
        {
            if (pixelState == PixelState.YELLOW_ONLY)
            {
                previousDriveState = driveState;
                driveState = DriveState.AWAY_FROM_SPIKE_MARK;
            }
        }
        else if (driveState == DriveState.AWAY_FROM_SPIKE_MARK)
        {
            if (!(drivePowered))
            {
                c4.followTrajectoryAsync(awayFromSpikeMark);
                drivePowered = true;
                driveTimer.reset();
            }
            else if (!(c4.isBusy()) && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED) && (wristState == WristState.FOLD))
            {
                previousDriveState = driveState;
                driveState = DriveState.TO_DETECTION_SPOT;
                currentPose = awayFromSpikeMark.end();
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.TO_DETECTION_SPOT)
        {
            if (!(drivePowered))
            {
                c4.followTrajectoryAsync(toAprilTagDetectionSpot);
                drivePowered = true;
            }
            else if (!(c4.isBusy()))
            {
                previousDriveState = driveState;
                driveState = DriveState.LOOKING_FOR_APRIL_TAG;
                currentPose = toAprilTagDetectionSpot.end();
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.LOOKING_FOR_APRIL_TAG)
        {
            if ((aprilTagState == AprilTagState.TARGET_FOUND) && (armState == ArmState.OUTTAKE)
                    && (wristState == WristState.DOWN_OUTTAKE) && (slideState == SlideState.LOW))
            {
                previousDriveState = driveState;
                driveState = DriveState.ADJUSTING_HORIZONTALLY_APRIL_TAG;
            }
        }
        else if (driveState == DriveState.ADJUSTING_HORIZONTALLY_APRIL_TAG)
        {
            if (!(drivePowered))
            {
                if ((horizontalErrorToAprilTag + BotValues.CAMERA_DISTANCE_TO_CLAW) > BotValues.BACKDROP_ALIGNMENT_RANGE)
                {
                    adjustHorizontally = c4.trajectoryBuilder(currentPose)
                            .strafeRight(horizontalErrorToAprilTag + BotValues.CAMERA_DISTANCE_TO_CLAW)
                            .build();
                    c4.followTrajectoryAsync(adjustHorizontally);
                    currentPose = adjustHorizontally.end();
                }
                else if ((horizontalErrorToAprilTag + BotValues.CAMERA_DISTANCE_TO_CLAW) < (-1 * BotValues.BACKDROP_ALIGNMENT_RANGE))
                {
                    adjustHorizontally = c4.trajectoryBuilder(currentPose)
                            .strafeLeft(-1 * (horizontalErrorToAprilTag + BotValues.CAMERA_DISTANCE_TO_CLAW))
                            .build();
                    c4.followTrajectoryAsync(adjustHorizontally);
                    currentPose = adjustHorizontally.end();
                }
                drivePowered = true;
            }
            else if (!(c4.isBusy())) // Maybe add a condition that checks if the robot is in line with April Tag after adjusting
            {
                previousDriveState = driveState;
                driveState = DriveState.ADJUSTING_VERTICALLY_APRIL_TAG;
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.ADJUSTING_VERTICALLY_APRIL_TAG)
        {
            if (!(drivePowered))
            {
                if ((verticalErrorToAprilTag - BotValues.BACKDROP_SAFETY_DISTANCE) > BotValues.BACKDROP_ALIGNMENT_RANGE)
                {
                    adjustVertically = c4.trajectoryBuilder(currentPose)
                            .forward(verticalErrorToAprilTag - BotValues.BACKDROP_SAFETY_DISTANCE)
                            .build();
                    c4.followTrajectoryAsync(adjustVertically);
                    currentPose = adjustVertically.end();
                }
                else if ((verticalErrorToAprilTag - BotValues.BACKDROP_SAFETY_DISTANCE) < (-1 * BotValues.BACKDROP_ALIGNMENT_RANGE))
                {
                    adjustVertically = c4.trajectoryBuilder(currentPose)
                            .back(BotValues.BACKDROP_SAFETY_DISTANCE - verticalErrorToAprilTag)
                            .build();
                    c4.followTrajectoryAsync(adjustVertically);
                    currentPose = adjustVertically.end();
                }
                drivePowered = true;
            }
            else if (!(c4.isBusy())) // Maybe add a condition that checks if the limit switches are triggered (touching the backdrop)
            {
                previousDriveState = driveState;
                driveState = DriveState.SCORE_YELLOW;
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.SCORE_YELLOW)
        {
            if ((pixelState == PixelState.NO_PIXELS) && (leftClawState == ClawState.CLOSED)
                    && (rightClawState == ClawState.CLOSED))
            {
                awayFromBackdrop = c4.trajectoryBuilder(currentPose)
                        .back(BotValues.backdropBackOutDistance)
                        .build();
                previousDriveState = driveState;
                driveState = DriveState.AWAY_FROM_BACKDROP;
            }
        }
        else if (driveState == DriveState.AWAY_FROM_BACKDROP)
        {
            if (!(drivePowered))
            {
                c4.followTrajectoryAsync(awayFromBackdrop);
                drivePowered = true;
                driveTimer.reset();
            }
            else if (!(c4.isBusy()) && (armState == ArmState.INTAKE) && (wristState == WristState.FOLD)
                    && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED)
                    && (slideState == SlideState.INITIAL))
            {
                previousDriveState = driveState;
                driveState = DriveState.PARKING;
                currentPose = awayFromBackdrop.end();
                drivePowered = false;
            }
        }
        else if (driveState == DriveState.PARKING)
        {
            if (!(drivePowered))
            {
                if (autoState == AutoState.RED_AUDIENCE)
                {
                    toParking = c4.trajectoryBuilder(currentPose)
                            .splineToConstantHeading(BotValues.redParkingLeft, Math.toRadians(90))
                            .build();
                }
                else if (autoState == AutoState.BLUE_BACKDROP)
                {
                    toParking = c4.trajectoryBuilder(currentPose)
                            .splineToConstantHeading(BotValues.blueParkingLeft, Math.toRadians(90))
                            .build();
                }
                else if (autoState == AutoState.BLUE_AUDIENCE)
                {
                    toParking = c4.trajectoryBuilder(currentPose)
                            .splineToConstantHeading(BotValues.blueParkingRight, Math.toRadians(90))
                            .build();
                }
                else
                {
                    toParking = c4.trajectoryBuilder(currentPose)
                            .splineToConstantHeading(BotValues.redParkingRight, Math.toRadians(90))
                            .build();
                }
                c4.followTrajectoryAsync(toParking);
                drivePowered = true;
            }
            else if (!(c4.isBusy()))
            {
                previousDriveState = driveState;
                driveState = DriveState.PARKED;
                currentPose = toParking.end();
                drivePowered = false;
            }
        }


        // Slides FSM
        if (slideState == SlideState.INITIAL)
        {
            if (slidesPowered)
            {
                c4.leftSlides.setPower(0);
                c4.rightSlides.setPower(0);
                c4.rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                c4.rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidesPowered = false;
            }
            else if ((driveState == DriveState.LOOKING_FOR_APRIL_TAG) && (armState == ArmState.OUTTAKE))
            {
                previousSlideState = slideState;
                slideState = SlideState.UP_LOW;
            }
        }
        else if (slideState == SlideState.UP_LOW)
        {
            if (!(c4.rightSlides.isBusy()) && (c4.rightSlides.getCurrentPosition()) != 0)
            {
                previousSlideState = slideState;
                slideState = SlideState.LOW;
            }
            else if (!(c4.rightSlides.isBusy()))
            {
                double distance = BotValues.LOW_SET_LINE;
                c4.rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                c4.rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidePos = (int)((distance / (BotValues.SLIDE_HUB_DIAMETER * Math.PI)) * BotValues.TICKS_PER_REV_312);
                c4.rightSlides.setTargetPosition(slidePos);
                c4.rightSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                // set power for moving up
                c4.leftSlides.setPower(BotValues.slideUpAutoPow);
                c4.rightSlides.setPower(BotValues.slideUpAutoPow);
                slidesPowered = true;
            }
        }
        else if (slideState == SlideState.LOW)
        {
            if (slidesPowered)
            {
                c4.leftSlides.setPower(0);
                c4.rightSlides.setPower(0);
                c4.rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                c4.rightSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slidesPowered = false;
            }
            else if ((driveState == DriveState.AWAY_FROM_BACKDROP) && (driveTimer.milliseconds() >= BotValues.AWAY_FROM_BACKDROP_TIME))
            {
                previousSlideState = slideState;
                slideState = SlideState.DOWN;
            }
        }
        else if (slideState == SlideState.DOWN)
        {
            if (c4.touchSensor.isPressed())
            {
                previousSlideState = slideState;
                slideState = SlideState.INITIAL;
            }
            else
            {
                if (!(slidesPowered))
                {
                    c4.leftSlides.setPower(BotValues.slideDownAutoPow);
                    c4.rightSlides.setPower(BotValues.slideDownAutoPow);
                    slidesPowered = true;
                }
            }
        }


        // Arm FSM
        if (armState == ArmState.INTAKE)
        {
            if ((driveState == DriveState.LOOKING_FOR_APRIL_TAG) && (wristState == WristState.INTAKE))
            {
                previousArmState = armState;
                armState = ArmState.TO_OUTTAKE;
            }
        }
        else if (armState == ArmState.TO_OUTTAKE)
        {
            if (!(armPowered))
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_OUTTAKE);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_OUTTAKE);
                armTimer.reset();
                armPowered = true;
            }
            else if (armTimer.milliseconds() >= BotValues.ARM_TO_OUTTAKE_TIME)
            {
                previousArmState = armState;
                armState = ArmState.OUTTAKE;
                armPowered = false;
            }
        }
        else if (armState == ArmState.OUTTAKE)
        {
            if ((driveState == DriveState.AWAY_FROM_BACKDROP) && (slideState == SlideState.INITIAL)
                    && (wristState == WristState.INTAKE) && (leftClawState == ClawState.CLOSED)
                    && (rightClawState == ClawState.CLOSED))
            {
                previousArmState = armState;
                armState = ArmState.TO_INTAKE;
            }
        }
        else if (armState == ArmState.TO_INTAKE)
        {
            if (!(armPowered))
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_HOME);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_HOME);
                armTimer.reset();
                armPowered = true;
            }
            else if (armTimer.milliseconds() >= BotValues.ARM_TO_INTAKE_TIME)
            {
                previousArmState = armState;
                armState = ArmState.INTAKE;
                armPowered = false;
            }
        }


        // Wrist FSM
        if (wristState == WristState.FOLD)
        {
            if ((driveState == DriveState.AT_SPIKE_MARK) && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED))
            {
                previousWristState = wristState;
                wristState = WristState.TO_INTAKE;
            }
            else if ((driveState == DriveState.LOOKING_FOR_APRIL_TAG) && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED))
            {
                previousWristState = wristState;
                wristState = WristState.TO_INTAKE;
            }
        }
        else if (wristState == WristState.TO_INTAKE)
        {
            if (!(wristPowered))
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_INTAKE);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_INTAKE);
                wristTimer.reset();
                wristPowered = true;
            }
            else if (wristTimer.milliseconds() >= BotValues.WRIST_TO_INTAKE_TIME)
            {
                previousWristState = wristState;
                wristState = WristState.INTAKE;
                wristPowered = false;
            }
        }
        else if (wristState == WristState.INTAKE)
        {
            if ((driveState == DriveState.AWAY_FROM_SPIKE_MARK) && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED))
            {
                previousWristState = wristState;
                wristState = WristState.TO_FOLD;
            }
            if ((driveState == DriveState.LOOKING_FOR_APRIL_TAG) && (armState == ArmState.OUTTAKE))
            {
                previousWristState = wristState;
                wristState = WristState.TO_DOWN_OUTTAKE;
            }
            else if ((driveState == DriveState.AWAY_FROM_BACKDROP) && (armState == ArmState.INTAKE) && (leftClawState == ClawState.CLOSED) && (rightClawState == ClawState.CLOSED))
            {
                previousWristState = wristState;
                wristState = WristState.TO_FOLD;
            }
        }
        else if (wristState == WristState.TO_DOWN_OUTTAKE)
        {
            if (!(wristPowered))
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_DOWN);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_DOWN);
                wristTimer.reset();
                wristPowered = true;
            }
            else if (wristTimer.milliseconds() >= BotValues.WRIST_TO_OUTTAKE_DOWN_TIME)
            {
                previousWristState = wristState;
                wristState = WristState.DOWN_OUTTAKE;
                wristPowered = false;
            }
        }
        else if (wristState == WristState.DOWN_OUTTAKE)
        {
            if ((driveState == DriveState.AWAY_FROM_BACKDROP) && (driveTimer.milliseconds() >= BotValues.AWAY_FROM_BACKDROP_TIME))
            {
                previousWristState = wristState;
                wristState = WristState.TO_INTAKE;
            }
        }
        else if (wristState == WristState.TO_FOLD)
        {
            if (!(wristPowered))
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_HOME);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_HOME);
                wristTimer.reset();
                wristPowered = true;
            }
            else if (wristTimer.milliseconds() >= BotValues.WRIST_TO_FOLD_TIME)
            {
                previousWristState = wristState;
                wristState = WristState.FOLD;
                wristPowered = false;
            }
        }


        // Left Claw FSM (Assuming purple outtake is left claw)
        if (leftClawState == ClawState.CLOSED)
        {
            if ((driveState == DriveState.AT_SPIKE_MARK) && (wristState == WristState.INTAKE))
            {
                previousLeftClawState = leftClawState;
                leftClawState = ClawState.TO_OPEN;
            }
        }
        else if (leftClawState == ClawState.TO_OPEN)
        {
            if (!(leftClawPowered))
            {
                c4.leftClaw.setPosition(BotValues.LEFT_CLAW_RANGE);
                leftClawTimer.reset();
                leftClawPowered = true;
            }
            else if (leftClawTimer.milliseconds() >= BotValues.LEFT_CLAW_OPEN_TIME)
            {
                previousLeftClawState = leftClawState;
                leftClawState = ClawState.OPEN;
                leftClawPowered = false;
            }
        }
        else if (leftClawState == ClawState.OPEN)
        {
            if ((pixelState == PixelState.YELLOW_ONLY) && (driveState == DriveState.AWAY_FROM_SPIKE_MARK)
                    && (driveTimer.milliseconds() >= BotValues.AWAY_FROM_SPIKE_MARK_TIME))
            {
                previousLeftClawState = leftClawState;
                leftClawState = ClawState.TO_CLOSED;
            }
        }
        else if (leftClawState == ClawState.TO_CLOSED)
        {
            if (!(leftClawPowered))
            {
                c4.leftClaw.setPosition(BotValues.LEFT_CLAW_HOME);
                leftClawTimer.reset();
                leftClawPowered = true;
            }
            else if (leftClawTimer.milliseconds() >= BotValues.LEFT_CLAW_CLOSE_TIME)
            {
                previousLeftClawState = leftClawState;
                leftClawState = ClawState.CLOSED;
                leftClawPowered = false;
            }
        }


        // Right Claw FSM (Assuming yellow outtake is right claw)
        if (rightClawState == ClawState.CLOSED)
        {
            if ((wristState == WristState.DOWN_OUTTAKE) && (armState == ArmState.OUTTAKE) && (driveState == DriveState.SCORE_YELLOW))
            {
                previousRightClawState = rightClawState;
                rightClawState = ClawState.TO_OPEN;
            }
        }
        else if (rightClawState == ClawState.TO_OPEN)
        {
            if (!(rightClawPowered))
            {
                c4.rightClaw.setPosition(BotValues.RIGHT_CLAW_RANGE);
                rightClawTimer.reset();
                rightClawPowered = true;
            }
            else if (rightClawTimer.milliseconds() >= BotValues.RIGHT_CLAW_OPEN_TIME)
            {
                previousRightClawState = rightClawState;
                rightClawState = ClawState.OPEN;
                rightClawPowered = false;
            }
        }
        else if (rightClawState == ClawState.OPEN)
        {
            if ((pixelState == PixelState.NO_PIXELS) && (driveState == DriveState.SCORE_YELLOW))
            {
                previousRightClawState = rightClawState;
                rightClawState = ClawState.TO_CLOSED;
            }
        }
        else if (rightClawState == ClawState.TO_CLOSED)
        {
            if (!(rightClawPowered))
            {
                c4.rightClaw.setPosition(BotValues.RIGHT_CLAW_HOME);
                rightClawTimer.reset();
                rightClawPowered = true;
            }
            else if (rightClawTimer.milliseconds() >= BotValues.RIGHT_CLAW_CLOSE_TIME)
            {
                previousRightClawState = rightClawState;
                rightClawState = ClawState.CLOSED;
                rightClawPowered = false;
            }
        }


        // Pixels FSM
        if (pixelState == PixelState.YELLOW_AND_PURPLE)
        {
            if ((leftClawState == ClawState.OPEN) && (driveState == DriveState.AT_SPIKE_MARK))
            {
                purplePixelTimer.reset();
                previousPixelState = pixelState;
                pixelState = PixelState.DROPPING_PURPLE;
            }
        }
        else if (pixelState == PixelState.DROPPING_PURPLE)
        {
            if (purplePixelTimer.milliseconds() >= BotValues.PURPLE_PIXEL_DROP_TIME)
            {
                previousPixelState = pixelState;
                pixelState = PixelState.YELLOW_ONLY;
            }
        }
        else if (pixelState == PixelState.YELLOW_ONLY)
        {
            if (rightClawState == ClawState.OPEN)
            {
                yellowPixelTimer.reset();
                previousPixelState = pixelState;
                pixelState = PixelState.DROPPING_YELLOW;
            }
        }
        else if (pixelState == PixelState.DROPPING_YELLOW)
        {
            if (yellowPixelTimer.milliseconds() >= BotValues.YELLOW_PIXEL_DROP_TIME)
            {
                previousPixelState = pixelState;
                pixelState = PixelState.NO_PIXELS;
            }
        }


        // April Tag FSM
        if (aprilTagState == AprilTagState.INITIAL)
        {
            if (driveState == DriveState.LOOKING_FOR_APRIL_TAG)
            {
                previousAprilTagState = aprilTagState;
                aprilTagState = AprilTagState.GETTING_DETECTIONS;
            }
        }
        else if (aprilTagState == AprilTagState.GETTING_DETECTIONS)
        {
            if (!(aprilTagDetectionsRequested))
            {
                currentDetections = aprilTagProcessor.getDetections();
                aprilTagTimer.reset();
                aprilTagDetectionsRequested = true;
            }
            else if (aprilTagTimer.milliseconds() >= BotValues.APRIL_TAG_DETECTION_TIME)
            {
                previousAprilTagState = aprilTagState;
                aprilTagState = AprilTagState.VERIFYING_DETECTIONS;
            }
        }
        else if (aprilTagState == AprilTagState.VERIFYING_DETECTIONS)
        {
            if (currentDetections == null || currentDetections.size() == 0)
            {
                previousAprilTagState = aprilTagState;
                aprilTagState = AprilTagState.REFRESHING_DETECTIONS;
                aprilTagDetectionsRequested = false;
            }
            else
            {
                telemetry.addData("# AprilTags Detected", currentDetections.size());
                telemetry.update();
                currentAprilTagIndex = 0;
                previousAprilTagState = aprilTagState;
                aprilTagState = AprilTagState.PROCESSING_DETECTIONS;
            }
        }
        else if (aprilTagState == AprilTagState.REFRESHING_DETECTIONS)
        {
            if (!(aprilTagDetectionsRequested))
            {
                currentDetections = aprilTagProcessor.getFreshDetections();
                aprilTagTimer.reset();
                aprilTagDetectionsRequested = true;
            }
            else if (aprilTagTimer.milliseconds() >= BotValues.APRIL_TAG_DETECTION_TIME)
            {
                previousAprilTagState = aprilTagState;
                aprilTagState = AprilTagState.VERIFYING_DETECTIONS;
            }
        }
        else if (aprilTagState == AprilTagState.PROCESSING_DETECTIONS)
        {
            if (!(detectionSelected))
            {
                if (currentAprilTagIndex < currentDetections.size())
                {
                    chosenDetection = currentDetections.get(currentAprilTagIndex);
                    detectionSelected = true;
                }
                else
                {
                    previousAprilTagState = aprilTagState;
                    aprilTagState = AprilTagState.REFRESHING_DETECTIONS;
                    aprilTagDetectionsRequested = false;
                }
            }
            else if ((chosenDetection.metadata != null))
            {
                if (chosenDetection.id == desiredAprilTagID)
                {
                    telemetry.addLine("ID: " + chosenDetection.id + ", " + chosenDetection.metadata.name);
                    telemetry.update();
                    previousAprilTagState = aprilTagState;
                    horizontalErrorToAprilTag = chosenDetection.ftcPose.x;
                    verticalErrorToAprilTag = chosenDetection.ftcPose.y;
                    aprilTagState = AprilTagState.TARGET_FOUND;
                }
                else
                {
                    telemetry.addLine("ID: " + chosenDetection.id + ", " + chosenDetection.metadata.name);
                    telemetry.update();
                    currentAprilTagIndex++;
                    detectionSelected = false;
                }
            }
            else
            {
                telemetry.addData("Unknown Label", chosenDetection.id);
                telemetry.update();
                currentAprilTagIndex++;
                detectionSelected = false;
            }
        }
        c4.update();
    }
}