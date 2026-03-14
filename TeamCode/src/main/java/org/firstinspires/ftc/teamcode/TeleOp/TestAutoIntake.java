package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.BinarySearch;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Auto Intake", group="Tests")
public class TestAutoIntake extends OpMode {
    Action looseIntakeAction;
    boolean actionActive = true;
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
        looseIntakeAction = new Robot.LooseIntakeAction(new Pose2d(-60, 40, 0));
    }

    @Override
    public void loop(){
        if(actionActive){
            actionActive = looseIntakeAction.run(new TelemetryPacket());
        }
    }
}
