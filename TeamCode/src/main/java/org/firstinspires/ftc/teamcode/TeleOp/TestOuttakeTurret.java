package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Outtake Turret", group="Tests")
public class TestOuttakeTurret extends OpMode {
    int mode = 0;
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        telemetry.addLine("A for manual mode, B for limelight aim mode, X for odometry");
        if(gamepad1.a){
            mode = 0;
        }
        if(gamepad1.b){
            mode = 1;
        }
        if(gamepad1.x){
            mode = 2;
        }
        if(mode == 0) {
            Robot.outtakeTurret.setPos(Robot.outtakeTurret.getPos() + gamepad1.left_stick_y);
            Robot.telemetry.addData("Outtake Turret Pos", Robot.outtakeTurret.getPos());
        }
        if(mode == 1) {
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
            double targetTurretPos = Math.toDegrees(Math.atan2(72-pose.position.y, -72-pose.position.x));
            Robot.telemetry.addData("Target Abs Turret Pos", targetTurretPos);
            Robot.telemetry.addData("Target Rel Turret Pos", targetTurretPos-Math.toDegrees(pose.heading.toDouble()));
            LLResult llResult;
            llResult = Robot.limelight.getLatestResult();
            // TODO see if apriltag matches our alliance
            if(llResult != null && llResult.isValid()){
                double xPos = llResult.getFiducialResults().get(0).getTargetXDegrees();
                Robot.telemetry.addData("Deg", xPos);
                Robot.outtakeTurret.setPos(Robot.outtakeTurret.getPos() + xPos);
            }
            Robot.telemetry.addData("Outtake Turret Pos", Robot.outtakeTurret.getPos());
        }
        if(mode == 2) {
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
            double targetTurretPos = Math.toDegrees(Math.atan2(72-pose.position.y, -72-pose.position.x));
            Robot.telemetry.addData("Target Abs Turret Pos", targetTurretPos);
            Robot.telemetry.addData("Target Rel Turret Pos", targetTurretPos-Math.toDegrees(pose.heading.toDouble()));
            Robot.outtakeTurret.setPos(targetTurretPos-Math.toDegrees(pose.heading.toDouble()));
            Robot.telemetry.addData("Outtake Turret Pos", Robot.outtakeTurret.getPos());
        }
    }
}
