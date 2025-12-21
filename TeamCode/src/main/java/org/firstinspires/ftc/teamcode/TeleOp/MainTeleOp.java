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

    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        Robot.drive.updatePoseEstimate();

        Vector2d driveVector = new Vector2d(0, 0);
        driveVector = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );
        if(FIELD_CENTRIC){
            driveVector = Robot.drive.localizer.getPose().heading.times(driveVector);
        }
        Robot.drive.setDrivePowers(new PoseVelocity2d(
                driveVector,
                -gamepad1.right_stick_x
        ));

        double power = gamepad1.a ? 1.0 : 0.0;
        Robot.intake.setPower(power);
        Robot.transferIn.setPower(power);
        Robot.transferUp.motor.setPower(power);
        Robot.telemetry.addData("Intake/Transfer Power", power);

        Robot.aimOuttakeTurret();
        if(gamepad1.y){
            Robot.shootOuttake();
        }
        else{
            Robot.outtake.setPos(0, 0);
        }
    }
}
