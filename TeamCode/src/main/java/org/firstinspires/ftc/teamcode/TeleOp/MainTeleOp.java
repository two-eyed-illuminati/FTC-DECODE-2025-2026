package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Main TeleOp", group="Main")
public class MainTeleOp extends OpMode {
    public static boolean FIELD_CENTRIC = true;
    public double currentMinOuttakeVel = 0.0;
    public double currentMaxOuttakeVel = 0.0;

    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        Robot.drive.updatePoseEstimate();

        Vector2d driveVector = new Vector2d(0, 0);
        if(FIELD_CENTRIC){
            double theta = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            if(Robot.alliance == Robot.Alliance.BLUE){
                theta -= Math.toRadians(90);
            }
            else{
                theta += Math.toRadians(90);
            }
            double mag = Math.sqrt(
                    gamepad1.left_stick_y*gamepad1.left_stick_y+
                    gamepad1.left_stick_x*gamepad1.left_stick_x);

            double newTheta = theta - Robot.drive.localizer.getPose().heading.log();
            driveVector = new Vector2d(
                    mag*Math.cos(newTheta),
                    mag*Math.sin(newTheta)
            );
        }
        else{
            driveVector = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
        }
        Robot.drive.setDrivePowers(new PoseVelocity2d(
                driveVector,
                -gamepad1.right_stick_x
        ));

        if(gamepad1.a){
            Robot.intake.setPower(1.0);
            Robot.transfer.setPos(0, 0.15*Robot.transfer.maxVel);
            Robot.aimOuttakeTurret();
            Robot.outtake.setPos(0, -1440.0);
        }
        else if(gamepad1.y){
            Robot.intake.setPower(1.0);

            double[] outtakeVels = Robot.shootOuttake();
            currentMinOuttakeVel = outtakeVels[0];
            currentMaxOuttakeVel = outtakeVels[1];

            if(currentMaxOuttakeVel >= Robot.outtake.getVel() && Robot.outtake.getVel() >= currentMinOuttakeVel) {
                Robot.transfer.setPos(0, 0.75*Robot.transfer.maxVel);
            }
            else{
                Robot.transfer.setPos(0, 0.0);
            }
            Robot.aimOuttakeTurret();
        }
        else if(gamepad1.x){
            Robot.intake.setPower(-1.0);
            Robot.transfer.setPos(0, -Robot.transfer.maxVel);
            Robot.aimOuttakeTurret();
        }
        else if(gamepad1.b){
            Robot.intake.setPower(-1.0);
            Robot.transfer.setPos(0, -Robot.transfer.maxVel);
            Robot.aimOuttakeTurret();
            Robot.outtake.setPos(0, -Robot.outtake.maxVel);
        }
        else{
            Robot.intake.setPower(0.0);
            Robot.transfer.setPos(0, 0.0);
            Robot.aimOuttakeTurret();
            if(Robot.drive.localizer.getPose().position.x + Math.abs(Robot.drive.localizer.getPose().position.y) < 10){
                double[] outtakeVels = Robot.shootOuttake();
                currentMinOuttakeVel = outtakeVels[0];
                currentMaxOuttakeVel = outtakeVels[1];
            }
            else{
                Robot.outtake.setPos(0, -1440.0);
                currentMinOuttakeVel = 0.0;
                currentMaxOuttakeVel = 0.0;
            }
        }
        Robot.telemetry.addData("Actual Intake Power", Robot.intake.getPower());
        Robot.telemetry.addData("Actual Transfer Up Vel (deg/s)", Robot.transfer.getVel());

        Robot.telemetry.update();
    }
}
