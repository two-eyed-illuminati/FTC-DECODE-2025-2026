package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class Robot{
  //Constants
  public static double START_X = -58.0586;
  public static double START_Y = -40.7964;
  public static double START_HEADING = -128.71;
  //Mechanisms, IMU, etc.
  public static MecanumDrive drive;
  public static DcMotorEx intake;
  public static ContinuousMotorMechanism transfer;
  public static MotorMechanism outtakeTurret;
  public static ContinuousMotorMechanism outtake;
  public static PIDFController outtakeController;
  public static Limelight3A limelight;
  public static MultipleTelemetry telemetry;
  //Stored Values
  public enum Alliance{
    BLUE, RED
  }
  public static Alliance alliance = Alliance.BLUE; //0 = blue, 1 = red
  public static Pose2d pose = new Pose2d(0, 0, 0);
  public static boolean initialized = false;

  public static void initialize(HardwareMap hardwareMap, Telemetry dsTelemetry){
    initializeOpMode(hardwareMap, dsTelemetry);

    if(!initialized) {
      drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
      intake = hardwareMap.get(DcMotorEx.class, "intake");
      DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
      transfer = new ContinuousMotorMechanism(transferMotor,
              360.0/145.1, 6900
      );
      DcMotorEx outtakeTurretMotor = hardwareMap.get(DcMotorEx.class, "outtakeTurret");
      outtakeTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      outtakeTurretMotor.setTargetPosition(0);
      outtakeTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      outtakeTurret = new MotorMechanism(outtakeTurretMotor,
              -90, 90, -537.7/4*4, 537.7/4*4, 1872);
      DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
      outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      outtake = new ContinuousMotorMechanism(outtakeMotor,
              360.0/28.0, 36000.0
      );
      outtakeController = new PIDFController(1.0/8000.0, 1.0/30345.0);
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(5);
      limelight.start();

      drive.localizer.setPose(new Pose2d(
              START_X,
              alliance == Alliance.BLUE ? START_Y : -START_Y,
              alliance == Alliance.BLUE ? Math.toRadians(START_HEADING) : -Math.toRadians(START_HEADING)
      ));

      telemetry.addLine("Robot successfully initialized");
    }
    else{
      telemetry.addLine("Robot using previous initialization state");
      telemetry.addLine("If you would like to uninitialize, run the \"Uninitialize Robot\" OpMode");
    }

    initialized = true;
    telemetry.update();
  }

  public static void initializeOpMode(HardwareMap hardwareMap, Telemetry dsTelemetry){
    telemetry = new MultipleTelemetry( //apparently needs to reinit each time opmode inits
            dsTelemetry, // Driver Station telemetry
            FtcDashboard.getInstance().getTelemetry() // Dashboard telemetry
    );
    if(initialized) {
      outtakeTurret.motor.setTargetPosition(outtake.motor.getCurrentPosition());
      outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      outtake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
      drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

      drive.localizer.setPose(pose);
    }
  }

  public static double artifactPos(double v0, double theta, double x){
    double v0x = v0*Math.cos(Math.toRadians(theta));
    double v0y = v0*Math.sin(Math.toRadians(theta));
    double a = 0.5*(-3.4448818898);
    double b = v0x;
    double t = (-b+Math.sqrt(b*b-4*a*(-x)))/(2*a);
    double y = 1.25+v0y*t+0.5*(-30.183727034)*t*t;

    return y;
  }

  public static void aimOuttakeTurret(Pose2d robotPose){
    telemetry.addData("X", robotPose.position.x);
    telemetry.addData("Y", robotPose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(robotPose.heading.toDouble()));
    double turretXOffset = 2.9*Math.cos(Math.toRadians(-150.0)+robotPose.heading.log());
    double turretYOffset = 2.9*Math.sin(Math.toRadians(-150.0)+robotPose.heading.log());
    double targetTurretPos = Math.toDegrees(
            Math.atan2(
                    (alliance == Alliance.BLUE ? -72.0 : 72.0)-robotPose.position.y-turretYOffset,
                    -72.0-robotPose.position.x-turretXOffset
            )
    );
    telemetry.addData("Turret X", robotPose.position.x+turretXOffset);
    telemetry.addData("Turret Y", robotPose.position.y+turretYOffset);
    telemetry.addData("Target Abs Turret Pos", targetTurretPos);
    telemetry.addData("Target Rel Turret Pos", targetTurretPos-Math.toDegrees(robotPose.heading.toDouble()));
    double angle = (targetTurretPos-Math.toDegrees(robotPose.heading.toDouble())) % 360.0;
    if(angle > 180.0){
      angle -= 360.0;
    }
    telemetry.addData("Target Angle", angle);
    if(Math.abs(Clamp.clamp(angle, outtakeTurret.minPos, outtakeTurret.maxPos)-angle) > 20){
      angle = outtakeTurret.getPos();
    }
    telemetry.addData("Target Angle After Unnecessary Motion Reduction", angle);
    outtakeTurret.setPos(angle);
    telemetry.addData("Outtake Turret Pos", outtakeTurret.getPos());
  }

  public static void aimOuttakeTurret(){
    Pose2d pose = drive.localizer.getPose();
    aimOuttakeTurret(pose);
  }

  public static double[] shootOuttake(Pose2d robotPose, boolean pid){
    double turretXOffset = 2.9*Math.cos(Math.toRadians(-150.0)+robotPose.heading.log());
    double turretYOffset = 2.9*Math.sin(Math.toRadians(-150.0)+robotPose.heading.log());
    double currDistance = Math.sqrt(
            Math.pow((alliance == Alliance.BLUE ? -72.0 : 72.0)-robotPose.position.y-turretYOffset, 2)+
            Math.pow(-72.0-robotPose.position.x-turretXOffset, 2)
    );
    telemetry.addData("Curr Distance (in)", currDistance);
    double maxArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> 46.0/12.0 < artifactPos(vel, 45.0, currDistance/12.0));
    telemetry.addData("Max Artifact Vel (ft/s)", maxArtifactVel);
    double maxOuttakeVel = 1.4*(maxArtifactVel/0.8);
    telemetry.addData("Max Outtake Vel (ft/s)", maxOuttakeVel);
    double maxOuttakeAngVel = maxOuttakeVel/(2*Math.PI*0.1181102362)*360.0;
    telemetry.addData("Max Outtake Ang Vel (deg/s)", maxOuttakeAngVel);
    double maxOuttakeAngVelInitial = maxOuttakeAngVel/0.740740741;
    telemetry.addData("Max Outtake Ang Vel Initial (deg/s)", maxOuttakeAngVelInitial);

    double minArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> 40.0/12.0 < artifactPos(vel, 45.0, currDistance/12.0));
    telemetry.addData("Min Artifact Vel (ft/s)", minArtifactVel);
    double minOuttakeVel = 1.4*(minArtifactVel/0.8);
    telemetry.addData("Min Outtake Vel (ft/s)", minOuttakeVel);
    double minOuttakeAngVel = minOuttakeVel/(2*Math.PI*0.1181102362)*360.0;
    telemetry.addData("Min Outtake Ang Vel (deg/s)", minOuttakeAngVel);
    double minOuttakeAngVelInitial = minOuttakeAngVel/0.740740741;
    telemetry.addData("Min Outtake Ang Vel Initial (deg/s)", minOuttakeAngVelInitial);

    double targetArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> 45.0/12.0 < artifactPos(vel, 45.0, currDistance/12.0));
    telemetry.addData("Target Artifact Vel (ft/s)", targetArtifactVel);
    double targetOuttakeVel = 1.4*(targetArtifactVel/0.8);
    telemetry.addData("Target Outtake Vel (ft/s)", targetOuttakeVel);
    double targetOuttakeAngVel = targetOuttakeVel/(2*Math.PI*0.1181102362)*360.0;
    telemetry.addData("Target Outtake Ang Vel (deg/s)", targetOuttakeAngVel);
    double targetOuttakeAngVelInitial = targetOuttakeAngVel/0.740740741;
    telemetry.addData("Target Outtake Ang Vel Initial (deg/s)", targetOuttakeAngVelInitial);
    if(pid) {
      double targetPower = outtakeController.getPower(outtake.getVel(), targetOuttakeAngVelInitial);
      telemetry.addData("Target Outtake Power", targetPower);
      outtake.motor.setPower(targetPower);
    }
    else{
      outtake.setPos(0, targetOuttakeAngVelInitial);
    }
    telemetry.addData("Actual Outtake Ang Vel (deg/s)", outtake.getVel());
    telemetry.addData("Actual Outtake Power", outtake.motor.getPower());

    return new double[]{minOuttakeAngVelInitial, maxOuttakeAngVelInitial};
  }

  public static double[] shootOuttake(Pose2d robotPose){
    return shootOuttake(robotPose, true);
  }

  public static double[] shootOuttake(){
    Pose2d pose = drive.localizer.getPose();
    return shootOuttake(pose, true);
  }

  public static class ShootSequenceAction implements Action {
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedSinceTimeStartAttemptToShoot = new ElapsedTime();
    boolean attemptingToShoot = false;
    boolean started = false;
    double time = 3.5;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      if(!started) {
        elapsedTime.reset();
        elapsedSinceTimeStartAttemptToShoot.reset();
        started = true;
      }

      if(elapsedTime.seconds() > time){
        intake.setPower(0);
        transfer.setPos(0, 0);
        outtake.setPos(0, 0);
        return false;
      }
      aimOuttakeTurret();
      double[] outtakeVels = shootOuttake();

      if(elapsedTime.seconds() < 0.2){
        attemptingToShoot = false;
      }
      else if(((outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1]) ||
              elapsedTime.seconds() > time - 0.3) &&
              elapsedSinceTimeStartAttemptToShoot.seconds() < 0.6
      ){
        if(!attemptingToShoot){
          elapsedSinceTimeStartAttemptToShoot.reset();
        }
        attemptingToShoot = true;
        intake.setPower(1.0);
        transfer.setPos(0, 0.65*transfer.maxVel);
      }
      else if(elapsedSinceTimeStartAttemptToShoot.seconds() % 1.0 < 0.175){
        attemptingToShoot = false;
        intake.setPower(-1.0);
        transfer.setPos(0, -transfer.maxVel);
      }
      else{
        attemptingToShoot = false;
        intake.setPower(1.0);
        transfer.setPos(0, (outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1]) ? transfer.maxVel : 0.0);
      }

      packet.put("Elapsed Time (s)", elapsedTime.seconds());
      packet.put("Outtake Vel (deg/s)", outtake.getVel());
      packet.put("Transfer Vel (deg/s)", transfer.getVel());
      packet.put("Min Outtake Vel (deg/s)", outtakeVels[0]);
      packet.put("Max Outtake Vel (deg/s)", outtakeVels[1]);
      return true;
    }
  }
}