package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Outtake Turret", group="Tests")
public class TestOuttakeTurret extends OpMode {
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        Robot.outtakeTurret.setPos(Robot.outtakeTurret.getPos() + gamepad1.left_stick_y);
        Robot.telemetry.addData("Outtake Turret Pos", Robot.outtakeTurret.getPos());
    }
}
