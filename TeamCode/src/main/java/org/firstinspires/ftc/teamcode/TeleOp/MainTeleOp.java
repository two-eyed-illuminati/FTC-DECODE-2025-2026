package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.List;

@TeleOp(name="Main TeleOp", group="Main")
public class MainTeleOp extends OpMode {
    public static boolean FIELD_CENTRIC = true;
    public double currentMinOuttakeVel = 0.0;
    public double currentMaxOuttakeVel = 0.0;
    List<LynxModule> allHubs;

    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        PoseVelocity2d currDriveVel = Robot.drive.updatePoseEstimate();
        currDriveVel = Rotation2d.fromDouble(Robot.drive.localizer.getPose().heading.log()).times(currDriveVel);

        Robot.telemetry.addData("Drive Heading (deg)", Math.toDegrees(Robot.drive.localizer.getPose().heading.toDouble()));
        Robot.telemetry.addData("Drive X (in)", Robot.drive.localizer.getPose().position.x);
        Robot.telemetry.addData("Drive Y (in)", Robot.drive.localizer.getPose().position.y);

        Robot.telemetry.addData("Drive Vel X (in/s)", currDriveVel.linearVel.x);
        Robot.telemetry.addData("Drive Vel Y (in/s)", currDriveVel.linearVel.y);

        Robot.telemetry.addData("Drive FL Power", Robot.drive.leftFront.getPower());
        Robot.telemetry.addData("Drive FR Power", Robot.drive.rightFront.getPower());
        Robot.telemetry.addData("Drive BL Power", Robot.drive.leftBack.getPower());
        Robot.telemetry.addData("Drive BR Power", Robot.drive.rightBack.getPower());

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
        double driveMultiplier = gamepad1.left_trigger > 0.8 ? 0.6 : 1.0;
        driveVector = driveVector.times(driveMultiplier);

        double rotation = (-gamepad1.right_stick_x) * driveMultiplier;
        Robot.drive.setDrivePowers(new PoseVelocity2d(
                driveVector,
                rotation
        ));

        if(gamepad1.a){
            Robot.telemetry.addData("Mode", "stop intake");
            Robot.intake.setPower(0.0);
            Robot.aimOuttakeTurret(currDriveVel);
            Robot.stopper.setPosition(Robot.STOPPER_CLOSED_POS);
        }
        else if(gamepad1.right_trigger > 0.8){
            Robot.telemetry.addData("Mode", "shoot");
            Robot.intake.setPower(1.0);

            double[] outtakeVels = Robot.shootOuttake(currDriveVel, true);
            currentMinOuttakeVel = outtakeVels[0];
            currentMaxOuttakeVel = outtakeVels[1];

            if(currentMaxOuttakeVel >= Robot.outtake.getVel() && Robot.outtake.getVel() >= currentMinOuttakeVel) {
                Robot.telemetry.addData("Can Shoot", 1);
                Robot.stopper.setPosition(Robot.STOPPER_OPEN_POS);
            }
            else{
                Robot.telemetry.addData("Can Shoot", 0);
            }
            Robot.aimOuttakeTurret(currDriveVel);
        }
        else if(gamepad1.x){
            Robot.telemetry.addData("Mode", "reverse");
            Robot.intake.setPower(-1.0);
            Robot.stopper.setPosition(Robot.STOPPER_OPEN_POS);
            Robot.aimOuttakeTurret(currDriveVel);
        }
        else if(gamepad1.b){
            Robot.telemetry.addData("Mode", "super reverse");
            Robot.intake.setPower(-1.0);
            Robot.aimOuttakeTurret(currDriveVel);
            Robot.stopper.setPosition(Robot.STOPPER_OPEN_POS);
            Robot.outtake.setPos(0, -Robot.outtake.maxVel);
        }
        else{
            Robot.telemetry.addData("Mode", "intake");
            Robot.intake.setPower(1.0);
            Robot.stopper.setPosition(Robot.STOPPER_CLOSED_POS);
            Robot.aimOuttakeTurret(currDriveVel);
            double LEAD_TIME = 0.25;
            Pose2d futureRobotPose = new Pose2d(Robot.drive.localizer.getPose().position.x + LEAD_TIME*currDriveVel.linearVel.x, Robot.drive.localizer.getPose().position.y + LEAD_TIME*currDriveVel.linearVel.y, Robot.drive.localizer.getPose().heading.toDouble());
            if(futureRobotPose.position.x + Math.abs(futureRobotPose.position.y) < 24){
                Robot.telemetry.addData("In Launch Zone", 1);
                double[] outtakeVels = Robot.shootOuttake(currDriveVel, true);
                currentMinOuttakeVel = outtakeVels[0];
                currentMaxOuttakeVel = outtakeVels[1];
            }
            else{
                Robot.telemetry.addData("In Launch Zone", 0);
                Robot.shootOuttake(currDriveVel, false);
                currentMinOuttakeVel = 0.0;
                currentMaxOuttakeVel = 0.0;
            }
        }

        if(Robot.voltageToDistance(Robot.frontDistanceSensor.getVoltage()) < Robot.FRONT_DISTANCE_SENSOR_DETECTION_THRESH){
            Robot.ledLeft.setPosition(0.50);
            Robot.ledRight.setPosition(0.50);
        }
        else{
            Robot.ledLeft.setPosition(0.28);
            Robot.ledRight.setPosition(0.28);
        }

        Robot.telemetry.addData("Actual Intake Power", Robot.intake.getPower());

        Robot.telemetry.update();
    }
}
