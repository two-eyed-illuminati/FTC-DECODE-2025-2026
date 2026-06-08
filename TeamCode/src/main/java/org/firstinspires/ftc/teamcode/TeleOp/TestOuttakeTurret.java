package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Clamp;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Outtake Turret", group="Tests")
public class TestOuttakeTurret extends OpMode {
    int mode = 0;
    double angle = 0;
    boolean direction = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void init(){
        Robot.initialize(hardwareMap, telemetry);
        angle = Robot.outtakeTurret.minPos;
    }

    @Override
    public void loop(){
        Robot.telemetry.addLine("A for manual mode, B for limelight aim mode, X for odometry, Y for PID test");
        if(gamepad1.a){
            mode = 0;
        }
        if(gamepad1.b){
            mode = 1;
        }
        if(gamepad1.x){
            Robot.outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mode = 2;
        }
        if(gamepad1.y){
            mode = 3;
        }
        if(mode == 0) {
            Robot.outtakeTurret.motor.setPower(gamepad1.left_stick_y);
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
            Robot.aimOuttakeTurret();
        }
        if(mode == 3){
            if(angle >= Robot.outtakeTurret.maxPos){
                direction = true;
                time.reset();
            }
            else if(angle <= Robot.outtakeTurret.minPos){
                direction = false;
                time.reset();
            }
            if(direction){
                angle -= time.seconds()*100;
            }
            else{
                angle += time.seconds()*100;
            }
            double targetPower = Robot.outtakeTurretController.getPower(Robot.outtakeTurret.getPos(), angle);
            Robot.telemetry.addData("Target Outtake Turret Power", targetPower);
            Robot.telemetry.addData("Target Angle", angle);
            Robot.telemetry.addData("Actual Angle", Robot.outtakeTurret.getPos());
            Robot.outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Robot.outtakeTurret.motor.setPower(targetPower);
        }
        Robot.telemetry.update();
    }
}
