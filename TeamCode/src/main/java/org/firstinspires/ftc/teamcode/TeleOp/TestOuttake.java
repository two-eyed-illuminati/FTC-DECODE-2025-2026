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
//        Robot.outtake.setPower(gamepad1.left_stick_y);
        Robot.outtake.setPos(0, Robot.outtake.maxVel);
        Robot.telemetry.addData("Outtake Power", Robot.outtake.motor.getPower());
        Robot.telemetry.addData("Outtake Vel (deg/s)", Robot.outtake.motor.getVelocity()*(360/28));
        Robot.telemetry.addData("Outtake Pos (deg)", Robot.outtake.getPos());
        Robot.telemetry.update();
//        Robot.outtake.setVelocity(235*360/28);
//        Robot.outtake.setVelocity(200*360/28);
    }
}
