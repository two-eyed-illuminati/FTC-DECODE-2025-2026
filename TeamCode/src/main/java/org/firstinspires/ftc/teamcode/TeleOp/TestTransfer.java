package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Transfer", group="Tests")
public class TestTransfer extends OpMode {
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Robot.transferIn.setPower(gamepad1.left_stick_x);
        Robot.transferUp.motor.setPower(gamepad1.left_stick_y);
        Robot.telemetry.addData("transferIn Power", Robot.transferIn.getPower());
        Robot.telemetry.addData("transferUp Power", Robot.transferUp.motor.getPower());
    }
}
