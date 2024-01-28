package org.firstinspires.ftc.teamcode.Autonomous.V2;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ImageRecognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.classifier.Classifications;

import java.io.File;
import java.util.List;

@Autonomous (name = "Red Front")
public class redFront extends LinearOpMode
{
    private final static double WHEEL_DIAMETER = 4.0; // in inches
    private final static double SLIDE_HUB_DIAMETER = 1.5; // in inches
    private final static double TICKS_PER_REV = 537.7;


    // Motors
    private DcMotorEx frontleft;
    private DcMotorEx backright;
    private DcMotorEx backleft;
    private DcMotorEx frontright;


    // Sensors
    //private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle;
    private TouchSensor touchSensor;


    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;
    private ElapsedTime timer = new ElapsedTime();


    // Essentially the main method
    public void runOpMode()
    {
        // Initialization
        initMotors();
        telemetry.addData("C4", "Ready");
        telemetry.update();
        waitForStart();
        timer.reset();
        while (timer.milliseconds() < 6000)
        {
            power(0.5);
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

        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
    }


    // Motor Methods
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

    public double driveProgress()
    {
        double fL = Math.abs(((double)frontleft.getCurrentPosition() / frontleft.getTargetPosition()));
        //double fR = Math.abs(((double)frontright.getCurrentPosition() / frontright.getTargetPosition()));
        double bL = Math.abs((double)backleft.getCurrentPosition() / backleft.getTargetPosition());
        //double bR = Math.abs(((double)backright.getCurrentPosition() / backright.getTargetPosition()));
        return ((fL + bL) / 2);
    }

    public void forward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Forward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        sleep(50);
        yPos += distance;
    }

    public void backward(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Backward", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        sleep(50);
        yPos -= distance;
    }

    public void left(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection((DcMotorEx.Direction.REVERSE));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Left", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        sleep(50);
        xPos -= distance;
    }

    public void right(double distance)
    {
        // Reset encoder counts to 0
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Sets current mode to using encoders
        frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the distance the motors travel
        frontleft.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        backleft.setTargetPosition((int)(-1*distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //frontright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        //backright.setTargetPosition((int)(distance / (WHEEL_DIAMETER*Math.PI) * TICKS_PER_REV));
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection((DcMotorEx.Direction.FORWARD));

        // Set the motors to move to the specified positions
        frontleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //frontright.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // This is if the extra math stuff doesn't work
        /*power(ROBOT_SPEED); // give power to motors
        while (frontleft.isBusy() || frontright.isBusy() || backleft.isBusy() || backright.isBusy())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
        }*/
        power(0.3); // Initial power applied
        double progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
        progress = Math.abs((progress / TICKS_PER_REV) * Math.PI); // number of inches traveled so far
        while (driveProgress() < 0.5 && opModeIsActive())
        {
            // Accelerates for 1/2 of the path
            telemetry.addData("Accelerating", progress);
            telemetry.update();
            power(acceleratorTransform(progress)); // experiment with numbers in acceleratorTransform method
            progress = (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition())) / 2.0;
            progress = Math.abs((progress / TICKS_PER_REV) * Math.PI);
        }

        // number of inches yet to be traveled
        double error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
        error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
        error /= 2.0;
        error = (error / TICKS_PER_REV) * Math.PI;
        while ((frontleft.isBusy() || backleft.isBusy()) && opModeIsActive())
        {
            telemetry.addData("Right", distance);
            telemetry.update();
            power(acceleratorTransform(error)); // experiment with numbers in acceleratorTransform method
            error = Math.abs(frontleft.getTargetPosition()) + Math.abs(backleft.getTargetPosition());
            error -= (Math.abs(frontleft.getCurrentPosition()) + Math.abs(backleft.getCurrentPosition()));
            error /= 2.0;
            error = (error / TICKS_PER_REV) * Math.PI;
        }
        // Motors should stop moving after encoders reach their target position, but if they don't
        // then just add some sort of stopper into the code


        // Cut off power
        power(0.0);
        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        backleft.setDirection(DcMotorEx.Direction.REVERSE);
        backright.setDirection(DcMotorEx.Direction.FORWARD);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        sleep(50);
        xPos += distance;
    }

    public double acceleratorTransform(double input)
    {
        return Math.tanh(0.36 + 0.05 * input);
        // Play around with these numbers until they work
        // 0.6 is where the maximum possible power for the robot should go
        // 0.25 is where the minimum possible power for the robot should go
        // 0.05 is how much the power should increase/decrease for every inch traveled
        // input should be the number of inches traveled or the number of inches away from the target
    }

}