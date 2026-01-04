package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Throughput", group="Tests")
public class TestThroughput extends OpMode {
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
       double power = gamepad1.x ? 1.0 : 0.0;
       Robot.intake.setPower(power);
       Robot.transfer.motor.setPower(power);
       double outtakePower = gamepad1.y ? 19000.0 : 0.0;
       Robot.outtake.setPos(0, outtakePower);
       Robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
               -gamepad1.right_stick_x
       ));
    }
}