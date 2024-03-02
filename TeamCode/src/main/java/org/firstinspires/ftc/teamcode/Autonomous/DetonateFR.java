package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
    private String modelName;
    private AprilTagProcessor aprilTagProcessor;
    private boolean targetFound;
    private AprilTagDetection desiredAprilTag;
    private List<Classifications> results;
    private List<Category> categories;
    private List<AprilTagDetection> currentDetections;
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

    // Trajectories
    Trajectory toLeftSpikeMarkRA, toCenterSpikeMarkRA, toRightSpikeMarkRA;
    Trajectory toLeftSpikeMarkRB, toCenterSpikeMarkRB, toRightSpikeMarkRB;
    Trajectory toLeftSpikeMarkBA, toCenterSpikeMarkBA, toRightSpikeMarkBA;
    Trajectory toLeftSpikeMarkBB, toCenterSpikeMarkBB, toRightSpikeMarkBB;
    Trajectory toBackdropRedLeft, toBackdropRedCenter, toBackdropRedRight;
    Trajectory toBackdropBlueLeft, toBackdropBlueCenter, toBackdropBlueRight;
    Trajectory toStack1, toStack2, toStack3, toStack4, toStack5, toStack6;
    Trajectory toRedAprilTagDetectionSpot, toBlueAprilTagDetectionSpot, lastTrajectory;
    Trajectory lineToRedAprilTagDetectionSpot, lineToBlueAprilTagDetectionSpot, lineToStack4;
    Trajectory toRedParkingLeft, toRedParkingRight, toBlueParkingLeft, toBlueParkingRight;


    // States
    private enum AutoState {RED_BACKDROP, RED_AUDIENCE, BLUE_BACKDROP, BLUE_AUDIENCE};
    private AutoState autoState;
    private enum DriveState
    {
        INITIAL, TO_SPIKE_MARK, AT_SPIKE_MARK, TO_DETECTION_SPOT, LOOKING_FOR_APRIL_TAG, PREPARE_YELLOW_OUTTAKE, SCORE_YELLOW,
        TO_STACK_1, TO_STACK_2, TO_STACK_3, TO_STACK_4, TO_STACK_5, TO_STACK_6, AT_STACK, TO_BACKDROP, AT_BACKDROP, PARKING, PARKED;
    }
    private DriveState driveState;
    // States
    private enum SlideState {INITIAL, UP_LOW, LOW, UP_MEDIUM, MEDIUM, UP_HIGH, HIGH, DOWN, UP_MANUAL, DOWN_MANUAL, STATIONARY};
    private SlideState slideState, previousSlideState;
    private enum ArmState {INTAKE, TO_OUTTAKE, OUTTAKE, TO_INTAKE};
    private ArmState armState;
    private enum WristState {FOLD, TO_INTAKE, INTAKE, TO_DOWN_OUTTAKE, DOWN_OUTTAKE, TO_UP_OUTTAKE, UP_OUTTAKE, TO_FOLD};
    private WristState wristState;
    private enum ClawState {OPEN, CLOSED};
    private ClawState leftClawState, rightClawState;
    private enum PixelState {NO_PIXELS, LEFT_ONLY, RIGHT_ONLY, FULL};
    private PixelState pixelState;


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        c4 = new ExplosiveTachyonicParticle(hardwareMap);
        finalizeAutoState();
        initCV();
        initStates();
        initDriveSequences();

        telemetry.addData("C4", "Ready");
        telemetry.update();

        // Recognize position of prop on spike marks during Init, takes the latest recognition
        while (opModeInInit())
        {
            recognizePosition();
            telemetry.addData("Recognition", label);
            telemetry.update();
        }
        desiredAprilTagID = label;
        finalizeDriveSequence(autoState, label);

        // Clear up memory
        classifier.clearImageClassifier();

        if (autoState == AutoState.RED_AUDIENCE)
        {
            while (opModeIsActive() && !(isStopRequested())) {triggerActionsRedAudience();}
        }
        else if (autoState == AutoState.BLUE_BACKDROP)
        {
            while (opModeIsActive() && !(isStopRequested())) {triggerActionsBlueBack();}
        }
        else if (autoState == AutoState.BLUE_AUDIENCE)
        {
            while (opModeIsActive() && !(isStopRequested())) {triggerActionsBlueAudience();}
        }
        else if (autoState == AutoState.RED_BACKDROP)
        {
            while (opModeIsActive() && !(isStopRequested())) {triggerActionsRedBack();}
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
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        portal = (new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(BotValues.RESOLUTION_WIDTH, BotValues.RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor)).build();
        portal.stopLiveView();
        if (autoState == AutoState.RED_AUDIENCE) {modelName = BotValues.RA_MODEL_NAME;}
        else if (autoState == AutoState.BLUE_BACKDROP) {modelName = BotValues.BB_MODEL_NAME;}
        else if (autoState == AutoState.BLUE_AUDIENCE) {modelName = BotValues.BA_MODEL_NAME;}
        else {modelName = BotValues.RB_MODEL_NAME;}
        classifier = new ImageRecognition(BotValues.INFERENCE_CONFIDENCE_THRESHOLD, 1, 3, 0, 0, modelName);
        frameNum = 0;
        label = -1;
        desiredAprilTagID = -1;
        targetFound = false;
        desiredAprilTag  = null;
        currentDetections = null;
    }

    public void initStates()
    {
        autoState = AutoState.RED_BACKDROP;
        driveState = DriveState.INITIAL;
        slideState = SlideState.INITIAL;
        armState = ArmState.INTAKE;
        wristState = WristState.FOLD;
        leftClawState = ClawState.CLOSED;
        rightClawState = ClawState.CLOSED;
        pixelState = PixelState.FULL;
    }

    public void initDriveSequences()
    {
        // Red Audience
        currentPose = BotValues.startPose;
        c4.setPoseEstimate(currentPose);
        toLeftSpikeMarkRA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.leftSpikeRA)
                .build();
        toCenterSpikeMarkRA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.centerSpikeRA)
                .build();
        toRightSpikeMarkRA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.rightSpikeRA)
                .build();
        toLeftSpikeMarkRB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.leftSpikeRB)
                .build();
        toCenterSpikeMarkRB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.centerSpikeRB)
                .build();
        toRightSpikeMarkRB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.rightSpikeRB)
                .build();
        toLeftSpikeMarkBA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.leftSpikeBA)
                .build();
        toCenterSpikeMarkBA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.centerSpikeBA)
                .build();
        toRightSpikeMarkBA = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.rightSpikeBA)
                .build();
        toLeftSpikeMarkBB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.leftSpikeBB)
                .build();
        toCenterSpikeMarkBB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.centerSpikeBB)
                .build();
        toRightSpikeMarkBB = c4.trajectoryBuilder(currentPose)
                .lineToConstantHeading(BotValues.rightSpikeBB)
                .build();
        lineToStack4 = c4.trajectoryBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(BotValues.stack4.getX(), BotValues.stack4.getY(), Math.toRadians(90)))
                .build();
        toRedAprilTagDetectionSpot = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.aprilTagDetectionSpotRed, Math.toRadians(90))
                .build();
        lineToRedAprilTagDetectionSpot = c4.trajectoryBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(BotValues.aprilTagDetectionSpotRed.getX(), BotValues.aprilTagDetectionSpotRed.getY(), Math.toRadians(90)))
                .build();
        toBlueAprilTagDetectionSpot = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.aprilTagDetectionSpotBlue, Math.toRadians(90))
                .build();
        lineToBlueAprilTagDetectionSpot = c4.trajectoryBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(BotValues.aprilTagDetectionSpotBlue.getX(), BotValues.aprilTagDetectionSpotBlue.getY(), Math.toRadians(90)))
                .build();
        toStack1 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack1, Math.toRadians(90))
                .build();
        toStack2 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack2, Math.toRadians(90))
                .build();
        toStack3 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack3, Math.toRadians(90))
                .build();
        toStack4 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack4, Math.toRadians(90))
                .build();
        toStack5 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack5, Math.toRadians(90))
                .build();
        toStack6 = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.stack6, Math.toRadians(90))
                .build();
        toBackdropRedLeft = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropRedLeft, Math.toRadians(90))
                .build();
        toBackdropRedCenter = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropRedCenter, Math.toRadians(90))
                .build();
        toBackdropRedRight = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropRedRight, Math.toRadians(90))
                .build();
        toBackdropBlueLeft = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropBlueLeft, Math.toRadians(90))
                .build();
        toBackdropBlueCenter = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropBlueCenter, Math.toRadians(90))
                .build();
        toBackdropBlueRight = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.backdropBlueRight, Math.toRadians(90))
                .build();
        toRedParkingLeft = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.redParking1, Math.toRadians(90))
                .build();
        toRedParkingRight = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.redParking2, Math.toRadians(90))
                .build();
        toBlueParkingLeft = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.blueParking2, Math.toRadians(90))
                .build();
        toBlueParkingRight = c4.trajectoryBuilder(currentPose)
                .splineToConstantHeading(BotValues.blueParking1, Math.toRadians(90))
                .build();
        TrajectorySequence sequence1 = c4.trajectorySequenceBuilder(currentPose)
                .addTrajectory(toLeftSpikeMarkRA)
                .build();
    }

    public void finalizeDriveSequence(AutoState stateOfAuto, int propPosition)
    {

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
            else if (detection.metadata != null && detection.id == desiredAprilTagID)
            {
                targetFound = true;
                telemetry.addLine("ID: " + detection.id + ", " + detection.metadata.name);
                desiredAprilTag = detection;
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
            if (label == BotValues.LEFT)
            {
                driveState = DriveState.TO_SPIKE_MARK;
                lastTrajectory = toLeftSpikeMarkRB;
                c4.followTrajectoryAsync(toLeftSpikeMarkRB);
            }
            else if (label == BotValues.CENTER)
            {
                driveState = DriveState.TO_SPIKE_MARK;
                lastTrajectory = toCenterSpikeMarkRB;
                c4.followTrajectory(toCenterSpikeMarkRB);
            }
            else if (label == BotValues.RIGHT)
            {
                driveState = DriveState.TO_SPIKE_MARK;
                lastTrajectory = toRightSpikeMarkRB;
                c4.followTrajectoryAsync(toRightSpikeMarkRB);
            }
        }
        else if (driveState == DriveState.TO_SPIKE_MARK)
        {
            if (!(c4.isBusy()))
            {
                driveState = DriveState.AT_SPIKE_MARK;
                currentPose = lastTrajectory.end();
            }
        }
        else if (driveState == DriveState.AT_SPIKE_MARK)
        {
            if (pixelState == PixelState.LEFT_ONLY)
            {
                driveState = DriveState.TO_DETECTION_SPOT;
                lastTrajectory = lineToRedAprilTagDetectionSpot;
                c4.followTrajectoryAsync(lineToRedAprilTagDetectionSpot);
            }
        }
        else if (driveState == DriveState.TO_DETECTION_SPOT)
        {
            if (!(c4.isBusy()))
            {
                driveState = DriveState.LOOKING_FOR_APRIL_TAG;
                currentPose = lastTrajectory.end();
            }
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
        else if (driveState == DriveState.TO_STACK_4)
        {

        }
        else if (driveState == DriveState.TO_STACK_5)
        {

        }
        else if (driveState == DriveState.TO_STACK_6)
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
        c4.update();
    }

    public void triggerActionsRedAudience()
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
        else if (driveState == DriveState.TO_STACK_4)
        {

        }
        else if (driveState == DriveState.TO_STACK_5)
        {

        }
        else if (driveState == DriveState.TO_STACK_6)
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
        c4.update();
    }

    public void triggerActionsBlueBack()
    {

    }

    public void triggerActionsBlueAudience()
    {

    }
}