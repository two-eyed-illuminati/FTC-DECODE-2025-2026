package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Autonomous(name="Auto Far Zone", group="Auto")
public class AutoFarZone extends LinearOpMode {
    public static double START_X = 64.0;
    public static double START_Y = -8.7964;
    public static double START_HEADING = 180.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.Alliance selectedAlliance = null;
        while(!isStarted()){
            Robot.telemetry.addLine("Press X for BLUE alliance, B for RED alliance");
            if(selectedAlliance == null) {
                Robot.telemetry.addLine("WARNING: No current alliance selected!");
            }
            else{
                Robot.telemetry.addData("Current Alliance", selectedAlliance == Robot.Alliance.BLUE ? "BLUE" : "RED");
            }
            Robot.telemetry.update();
            if(gamepad1.x){
                selectedAlliance = Robot.Alliance.BLUE;
            }
            else if(gamepad1.b){
                selectedAlliance = Robot.Alliance.RED;
            }
        }
        if(selectedAlliance != null){
            Robot.alliance = selectedAlliance;
        }
        else{
            return;
        }

        PoseMap poseMap = Robot.alliance == Robot.Alliance.BLUE ? new IdentityPoseMap() :
                pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());

        Robot.drive.localizer.setPose(Robot.alliance == Robot.Alliance.BLUE ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble()));

        TrajectoryActionBuilder forward = Robot.drive.actionBuilder(startPose, poseMap).strafeTo(new Vector2d(START_X-5.0, START_Y));
        TrajectoryActionBuilder lateral = forward.fresh().strafeTo(new Vector2d(START_X-5.0, -32.0));

        Actions.runBlocking(
                new SequentialAction(
                    forward.build(),
                    lateral.build()
                )
        );
    }
}
