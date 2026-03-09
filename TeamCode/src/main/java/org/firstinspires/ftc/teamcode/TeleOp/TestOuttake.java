package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Outtake", group="Tests")
public class TestOuttake extends OpMode {
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
//        Robot.outtake.motor.setPower(gamepad1.x ? 0.4 : 0);
        double targetPower = Robot.outtakeController.getPower(Robot.outtake.getVel(), 20000.0);
        telemetry.addData("Target Outtake Power", targetPower);
        Robot.outtake.motor.setPower(targetPower);
        Robot.telemetry.addData("Gamepad", gamepad1.x);
        Robot.telemetry.addData("Gamepad", gamepad1.left_stick_y);
        Robot.telemetry.addData("Outtake Power", Robot.outtake.motor.getPower());
        Robot.telemetry.addData("Outtake Vel (deg/s)", Robot.outtake.getVel());
        Robot.telemetry.addData("Outtake Pos (deg)", Robot.outtake.getPos());
        Robot.telemetry.update();
//        Robot.outtake.setVelocity(235*360/28);
//        Robot.outtake.setVelocity(200*360/28);
    }
}
