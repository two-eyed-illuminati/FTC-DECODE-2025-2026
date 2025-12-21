package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.BinarySearch;
import org.firstinspires.ftc.teamcode.utils.Robot;

public class TestDynamicDistanceShoot extends OpMode {
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        Robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
        Robot.drive.updatePoseEstimate();
        Pose2d pose = Robot.drive.localizer.getPose();
        Robot.telemetry.addData("x", pose.position.x);
        Robot.telemetry.addData("y", pose.position.y);
        Robot.telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        double currDistance = Math.sqrt(Math.pow(72-pose.position.y+1.25, 2)+Math.pow(-72-pose.position.x+4.5, 2));
        double targetArtifactVel = BinarySearch.binarySearch(0, 1000,
                (vel) -> 40 > Robot.artifactPos(vel, 45, currDistance));
        Robot.telemetry.addData("Target Artifact Vel (ft/s)", targetArtifactVel);
        double targetOuttakeVel = (targetArtifactVel/0.8);
        Robot.telemetry.addData("Target Outtake Vel (ft/s)", targetOuttakeVel);
        double targetOuttakeAngVel = targetOuttakeVel/(2*Math.PI*0.1181102362)*360;
        Robot.telemetry.addData("Target Outtake Ang Vel (deg/s)", targetOuttakeAngVel);
        double targetOuttakeAngVelInitial = targetOuttakeAngVel/0.740740741;
        Robot.telemetry.addData("Target Outtake Ang Vel Initial (deg/s)", targetOuttakeAngVelInitial);
        Robot.outtake.setVelocity(targetOuttakeAngVelInitial);
    }
}
