package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.ImageRecognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;


@Autonomous(name = "Radium")
public class Radium extends LinearOpMode
{
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;
    private DcMotorEx intake;
    private DcMotorEx slides;
    private Servo boxTurner;
    private Servo boxOpener;
    private final static double BOX_TURNER_HOME = 0.5;
    private final static double BOX_TURNER_RANGE = 0.28;
    private final static double BOX_OPENER_HOME = 0.5;
    private final static double BOX_OPENER_RANGE = 0.108;
    private final int RESOLUTION_WIDTH = 1280;
    private final int RESOLUTION_HEIGHT = 720;
    private static final int DESIRED_TAG_ID = 2;
    private static final double BACKDROP_SAFETY_DISTANCE = 12.0; // in inches
    private static final String MODEL_NAME = "redBack.tflite";
    private ElapsedTime timer;
    private VisionPortal portal;
    private CameraName camera;
    private ImageRecognition classifier;
    private AprilTagProcessor aprilTagProcessor;
    private boolean targetFound;
    private AprilTagDetection desiredTag;
    private int label, frameNum;
    private long inferenceTime;

    @Override
    public void runOpMode()
    {
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
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slides.setDirection(DcMotorEx.Direction.REVERSE);

        boxTurner = hardwareMap.get(Servo.class, "box turner");
        boxTurner.setDirection(Servo.Direction.FORWARD);
        boxTurner.setPosition(BOX_TURNER_HOME);

        boxOpener = hardwareMap.get(Servo.class, "box opener");
        boxOpener.setDirection(Servo.Direction.FORWARD);
        boxOpener.setPosition(BOX_OPENER_HOME);

        File dir = new File("/sdcard/VisionPortal-Capture");
        String[] children = dir.list();
        for (int i = 0; i < children.length; i++)
        {
            boolean deleted = new File(dir, children[i]).delete();
            telemetry.addData("File Delete", "" + deleted);
            telemetry.update();
        }
        timer = new ElapsedTime();
        camera = hardwareMap.get(WebcamName.class, "cool cam");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor.setDecimation(2);
        portal = new VisionPortal.Builder().setCamera(camera).setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT)).addProcessor(aprilTagProcessor).build();
        portal.stopLiveView();
        classifier = new ImageRecognition(0.5f, 1, 3, 0, 0, MODEL_NAME);
        frameNum = 0;
        label = 0;
        List<Classifications> results;
        List<Category> categories;
        targetFound = false;
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = null;
        telemetry.addData("Camera", "Ready");
        telemetry.update();
        timer.reset();

        while (opModeInInit())
        {
            ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
            sleep(500);
            File input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
            sleep(500);
            Bitmap bitmap = classifier.PNG2BMP(input);
            sleep(500);
            results = classifier.classify(bitmap, 0);
            telemetry.addData("Results", results);
            telemetry.update();
            while (results == null && opModeIsActive())
            {
                telemetry.addData("Recognition", "Null");
                telemetry.update();
                frameNum++;
                ((VisionPortalImpl)portal).saveNextFrameRaw("Capture/" + frameNum);
                sleep(500);
                input = new File("/sdcard/VisionPortal-Capture/" + frameNum + ".png");
                sleep(500);
                bitmap = classifier.PNG2BMP(input);
                sleep(500);
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
        classifier.clearImageClassifier();
        if (label == 0)
        {
            telemetry.addData("Recognition", "Left");
            telemetry.update();
        }
        else if (label == 1)
        {
            telemetry.addData("Recognition", "Middle");
            telemetry.update();
        }
        else
        {
            telemetry.addData("Recognition", "Right");
            telemetry.update();
        }


        currentDetections = aprilTagProcessor.getDetections();
        sleep(250);
        while (currentDetections == null)
        {
            currentDetections = aprilTagProcessor.getFreshDetections();
            sleep(250);
        }
        while (currentDetections.get(0).id != DESIRED_TAG_ID)
        {
            frontleft.setPower(-0.25);
            backleft.setPower(0.25);
            backright.setPower(0.25);
            frontright.setPower(-0.25);
        }
        power(0);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null && !(targetFound))
            {
                if (detection.id == DESIRED_TAG_ID)
                {
                    targetFound = true;
                    desiredTag = detection;
                }
                else
                {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    telemetry.update();
                }
            }
            else if (targetFound) {break;}
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                telemetry.update();
            }
        }
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", desiredTag.ftcPose.pitch, desiredTag.ftcPose.roll, desiredTag.ftcPose.yaw));
            double  rangeError      = (desiredTag.ftcPose.range - BACKDROP_SAFETY_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            telemetry.update();
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();
    }
    public VisionPortalImpl getPortal() {return (VisionPortalImpl) portal;}
    public CameraName getCamera() {return camera;}

    private void power(double pow)
    {
        frontleft.setPower(pow);
        frontright.setPower(pow);
        backleft.setPower(pow);
        backright.setPower(pow);
    }

    private void power(double fL, double fR, double bL, double bR)
    {
        frontleft.setPower(fL);
        frontright.setPower(fR);
        backleft.setPower(bL);
        backright.setPower(bR);
    }

    private void left(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));
        backleft.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        frontright.setTargetPosition((int)(distance / (4*Math.PI) * 537.7));
        backright.setTargetPosition((int)(-1*distance / (4*Math.PI) * 537.7));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        power(0.5); // Initial power applied
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            power(0.5);
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code

        // Cut off power
        power(0.0);
        sleep(50);
    }
}