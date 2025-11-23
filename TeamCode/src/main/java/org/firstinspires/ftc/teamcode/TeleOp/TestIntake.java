package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Intake", group="Tests")
public class TestIntake extends OpMode {
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Robot.intake.setPower(gamepad1.left_stick_y);
        Robot.telemetry.addData("Intake Power", Robot.intake.getPower());
    }
}
