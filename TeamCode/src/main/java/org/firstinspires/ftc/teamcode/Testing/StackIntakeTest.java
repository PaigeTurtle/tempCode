package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.ExplosiveTachyonicParticle;
import org.firstinspires.ftc.teamcode.TeleOp.BotValues;

@TeleOp (name = "Stack Intake Test")
public class StackIntakeTest extends LinearOpMode
{
    private ExplosiveTachyonicParticle c4;
    @Override
    public void runOpMode()
    {
        c4 = new ExplosiveTachyonicParticle(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !(isStopRequested()))
        {
            if (gamepad1.dpad_up)
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_STACK_45);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_STACK_45);
            }
            else if (gamepad1.dpad_left)
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_STACK_34);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_STACK_34);
            }
            else if (gamepad1.dpad_right)
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_STACK_23);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_STACK_23);
            }
            else if (gamepad1.dpad_down)
            {
                c4.leftArm.setPosition(BotValues.LEFT_ARM_STACK_12);
                c4.rightArm.setPosition(BotValues.RIGHT_ARM_STACK_12);
            }

            if (gamepad1.y)
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_45);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_45);
            }
            else if (gamepad1.x)
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_34);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_34);
            }
            else if (gamepad1.b)
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_23);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_23);
            }
            else if (gamepad1.a)
            {
                c4.leftWrist.setPosition(BotValues.LEFT_WRIST_STACK_12);
                c4.rightWrist.setPosition(BotValues.RIGHT_WRIST_STACK_12);
            }
        }
    }
}
