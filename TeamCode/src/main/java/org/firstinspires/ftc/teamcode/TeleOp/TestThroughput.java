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
    int mode = 0;
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
       double power = gamepad1.x ? 1.0 : 0.0;
       Robot.intake.setPower(power);
       Robot.transfer.setPower(0.4*power);
       Robot.outtake.setPower(1.0);
    }
}
