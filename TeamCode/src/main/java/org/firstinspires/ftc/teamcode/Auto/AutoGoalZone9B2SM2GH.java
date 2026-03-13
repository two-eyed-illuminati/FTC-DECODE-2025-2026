package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Autonomous(name="Auto Goal Zone 9B 2SM 2GH", group="Main")
@Config
public class AutoGoalZone9B2SM2GH extends LinearOpMode {
    double START_X = -49.0;
    double START_Y = -50.5;
    double START_HEADING = -126.5;
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

        AutoBuilder autoBuilder = new AutoBuilder(Robot.drive.actionBuilder(startPose, poseMap));

        autoBuilder
                .goToCloseShoot("strafe", "")
                .shoot();
        autoBuilder
                .goToSpike1("")
                .intakeSpike1()
                .goToGateHit("left")
                .goToCloseShoot("strafe", "")
                .shoot();
        autoBuilder
                .goToSpike2()
                .intakeSpike2()
                .backUpAfterSpike2()
                .goToGateHit("right")
                .goToCloseShoot("strafe", "")
                .shoot();
        autoBuilder
                .leaveZone();

        Actions.runBlocking(
                autoBuilder.build()
        );
    }
}
