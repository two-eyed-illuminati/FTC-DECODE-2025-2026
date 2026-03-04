package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Stopper", group="Tests")
public class TestStopper extends OpMode {
    public boolean lastX = false;
    public boolean lastY = false;
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        double currPos = Robot.stopper.getPosition();
        double change = 0.0;
        if(!lastX && gamepad1.x){
            change = 0.01;
        }
        else if(!lastY && gamepad1.y){
            change = -0.01;
        }
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        Robot.stopper.setPosition(currPos + change);
        Robot.telemetry.addData("Pos", Robot.stopper.getPosition());
    }
}
