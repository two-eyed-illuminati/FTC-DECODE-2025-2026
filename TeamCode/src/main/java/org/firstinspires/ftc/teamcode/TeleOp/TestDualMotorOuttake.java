package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Dual Motor Outtake", group="Tests")
public class TestDualMotorOuttake extends OpMode {
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Robot.outtakeMotors.motor1.setPower(gamepad1.x ? 0.4 : 0);
        Robot.outtakeMotors.motor2.setPower(gamepad1.b ? 0.4 : 0);
        Robot.telemetry.addLine("X to turn motor 1, B to turn motor 2");
        Robot.telemetry.addData("Motor 1 Power", Robot.outtakeMotors.motor1.getPower());
        Robot.telemetry.addData("Motor 2 Power", Robot.outtakeMotors.motor2.getPower());
    }
}
