package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Shoot Sequence", group="Tests")
public class TestShootSequence extends OpMode {
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        if(gamepad1.y){
            Robot.shootSequence();
        }
    }
}
