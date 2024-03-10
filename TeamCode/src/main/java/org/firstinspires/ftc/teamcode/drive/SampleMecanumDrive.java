package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive
{
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(-0.04, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(BotValues.MAX_VEL, BotValues.MAX_ANG_VEL, BotValues.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(BotValues.MAX_ACCEL);

    private TrajectoryFollower follower;

    private List<DcMotorEx> driveMotors;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    // Motors
    public DcMotorEx frontleft, backright, backleft, frontright;
    public DcMotorEx leftSlides, rightSlides;
    public DcMotorEx leftHanger, rightHanger;
    public Servo leftArm, rightArm;
    public Servo leftWrist, rightWrist;
    public Servo leftClaw, rightClaw;
    public Servo planeLauncher;

    // Sensors
    public IMU imu;
    public Orientation lastAngles;
    public double currentAngle;
    public TouchSensor touchSensor;
    public ElapsedTime timer;
    public DistanceSensor distanceSensor;
    public static VoltageSensor voltageSensor;

    // Positions
    private double xPos; // in inches
    private double yPos; // in inches
    private int slidePos;

    public SampleMecanumDrive(HardwareMap hardwareMap)
    {
        super(BotValues.kV, BotValues.kA, BotValues.kStatic, BotValues.TRACK_WIDTH, BotValues.TRACK_WIDTH, LATERAL_MULTIPLIER);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        initMotors(hardwareMap);
        initSensors(hardwareMap);

        driveMotors = Arrays.asList(frontleft, backleft, backright, frontright);

        for (DcMotorEx motor : driveMotors)
        {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (BotValues.RUN_USING_ENCODER) {setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (BotValues.RUN_USING_ENCODER && BotValues.MOTOR_VELO_PID != null)
        {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, BotValues.MOTOR_VELO_PID);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, voltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }


    // Road Runner
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose)
    {
        return (new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed)
    {
        return (new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading)
    {
        return (new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT));
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose)
    {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                BotValues.MAX_ANG_VEL, BotValues.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle)
    {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory)
    {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence)
    {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update()
    {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {while (!Thread.currentThread().isInterrupted() && isBusy()) {update();}}

    public boolean isBusy() {return trajectorySequenceRunner.isBusy();}

    public void setMode(DcMotor.RunMode runMode) {for (DcMotorEx motor : driveMotors) {motor.setMode(runMode);}}

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (DcMotorEx motor : driveMotors) {motor.setZeroPowerBehavior(zeroPowerBehavior);}
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients)
    {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / voltageSensor.getVoltage()
        );

        for (DcMotorEx motor : driveMotors) {motor.setPIDFCoefficients(runMode, compensatedCoefficients);}
    }

    public void setWeightedDrivePower(Pose2d drivePower)
    {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1)
        {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions()
    {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : driveMotors)
        {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(BotValues.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities()
    {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : driveMotors)
        {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(BotValues.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public double getRawExternalHeading() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    @Override
    public Double getExternalHeadingVelocity()
    {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth)
    {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel)
    {
        return new ProfileAccelerationConstraint(maxAccel);
    }


    // Custom methods
    public void setMotorPowers(double fL, double bL, double bR, double fR) {power(fL, fR, bL, bR);}

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

    public void initMotors(HardwareMap hardwareMap)
    {
        // Initialize Motors
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));

        backright = hardwareMap.get(DcMotorEx.class, "back right");
        backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
        frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.REVERSE);
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

    public void initSensors(HardwareMap hardwareMap)
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

    public void forwardSlow() {power(BotValues.autoSlowPow);}
    public void backwardSlow() {power(-1 * BotValues.autoSlowPow);}
    public void strafeRightSlow() {power(BotValues.autoSlowPow, -1 * BotValues.autoSlowPow, -1 * BotValues.autoSlowPow, BotValues.autoSlowPow);}
    public void strafeLeftSlow() {power(-1 * BotValues.autoSlowPow, BotValues.autoSlowPow, BotValues.autoSlowPow, -1 * BotValues.autoSlowPow);}
    public void stopDrive() {power(0);}
    public void forwardFast() {power(BotValues.autoFastPow);}
    public void backwardFast() {power(-1 * BotValues.autoFastPow);}
    public void strafeRightFast() {power(BotValues.autoFastPow, -1 * BotValues.autoFastPow, -1 * BotValues.autoFastPow, BotValues.autoFastPow);}
    public void strafeLeftFast() {power(-1 * BotValues.autoFastPow, BotValues.autoFastPow, BotValues.autoFastPow, -1 * BotValues.autoFastPow);}

    public void openLeftClaw() {leftClaw.setPosition(BotValues.LEFT_CLAW_RANGE);}
    public void closeLeftClaw() {leftClaw.setPosition(BotValues.LEFT_CLAW_HOME);}
    public void openRightClaw() {leftClaw.setPosition(BotValues.RIGHT_CLAW_RANGE);}
    public void closeRightClaw() {leftClaw.setPosition(BotValues.RIGHT_CLAW_HOME);}
    public void wristToIntake()
    {
        leftWrist.setPosition(BotValues.LEFT_WRIST_INTAKE);
        rightWrist.setPosition(BotValues.RIGHT_WRIST_INTAKE);
    }
    public void wristFold()
    {
        leftWrist.setPosition(BotValues.LEFT_WRIST_HOME);
        rightWrist.setPosition(BotValues.RIGHT_WRIST_HOME);
    }
    public void wristToDownOuttake()
    {
        leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_DOWN);
        rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_DOWN);
    }
    public void wristToUpOuttake()
    {
        leftWrist.setPosition(BotValues.LEFT_WRIST_OUTTAKE_UP);
        rightWrist.setPosition(BotValues.RIGHT_WRIST_OUTTAKE_UP);
    }
    public void armToIntake()
    {
        leftArm.setPosition(BotValues.LEFT_ARM_HOME);
        rightArm.setPosition(BotValues.RIGHT_ARM_HOME);
    }
    public void armToOuttake()
    {
        leftArm.setPosition(BotValues.LEFT_ARM_OUTTAKE);
        rightArm.setPosition(BotValues.RIGHT_ARM_OUTTAKE);
    }
}
