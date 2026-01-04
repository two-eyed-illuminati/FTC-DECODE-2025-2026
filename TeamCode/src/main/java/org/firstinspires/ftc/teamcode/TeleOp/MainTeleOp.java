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

        double requestedPower = gamepad1.a ? 1.0 : (gamepad1.x ? -1.0 : 0.0);
        double upPower = Robot.outtake.getVel() >= currentMinOuttakeVel ? requestedPower : 0.0;
        Robot.intake.setPower(requestedPower);
        Robot.transferIn.setPower(requestedPower);
        Robot.transferUp.setPos(0, Robot.transferUp.maxVel*upPower);
        Robot.telemetry.addData("Intake/Transfer Power", requestedPower);
        Robot.telemetry.addData("Target Transfer Up Vel", Robot.transferUp.maxVel*upPower);
        Robot.telemetry.addData("Actual Transfer Up Vel (deg/s)", Robot.transferUp.getVel());

        Robot.aimOuttakeTurret();
        if(gamepad1.y){
            currentMinOuttakeVel = Robot.shootOuttake();
        }
        else{
            Robot.outtake.setPos(0, 0);
            currentMinOuttakeVel = 0.0;
        }

        Robot.telemetry.update();
    }
}
