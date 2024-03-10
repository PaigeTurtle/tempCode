package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.ExplosiveTachyonicParticle;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;

@TeleOp (name = "Stack Intake Test")
public class StackIntakeTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Servo leftArm = hardwareMap.get(Servo.class, "left arm");
        Servo rightArm = hardwareMap.get(Servo.class, "right arm");
        Servo leftWrist = hardwareMap.get(Servo.class, "left wrist");
        Servo rightWrist = hardwareMap.get(Servo.class, "right wrist");
        waitForStart();
        while (opModeIsActive() && !(isStopRequested()))
        {
            if (gamepad1.left_bumper)
            {
                leftArm.setPosition(BotValues.LEFT_ARM_STACK_5);
                rightArm.setPosition(BotValues.RIGHT_ARM_STACK_5);
            }
            else if (gamepad1.dpad_up)
            {
                leftArm.setPosition(BotValues.LEFT_ARM_STACK_45);
                rightArm.setPosition(BotValues.RIGHT_ARM_STACK_45);
            }
            else if (gamepad1.dpad_left)
            {
                leftArm.setPosition(BotValues.LEFT_ARM_STACK_34);
                rightArm.setPosition(BotValues.RIGHT_ARM_STACK_34);
            }
            else if (gamepad1.dpad_right)
            {
                leftArm.setPosition(BotValues.LEFT_ARM_STACK_23);
                rightArm.setPosition(BotValues.RIGHT_ARM_STACK_23);
            }
            else if (gamepad1.dpad_down)
            {
                leftArm.setPosition(BotValues.LEFT_ARM_STACK_12);
                rightArm.setPosition(BotValues.RIGHT_ARM_STACK_12);
            }

            if (gamepad1.y)
            {
                leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_45);
                rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_45);
            }
            else if (gamepad1.x)
            {
                leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_34);
                rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_34);
            }
            else if (gamepad1.b)
            {
                leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_23);
                rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_23);
            }
            else if (gamepad1.a)
            {
                leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_12);
                rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_12);
            }else if (gamepad1.right_bumper)
            {
                leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_5);
                rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_5);
            }
        }
    }
}
