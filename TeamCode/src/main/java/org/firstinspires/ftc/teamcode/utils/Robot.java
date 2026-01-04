package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class Robot{
  //Constants

  //Mechanisms, IMU, etc.
  public static MecanumDrive drive;
  public static DcMotorEx intake;
  public static ContinuousMotorMechanism transfer;
  public static MotorMechanism outtakeTurret;
  public static ContinuousMotorMechanism outtake;
  public static Limelight3A limelight;
  public static MultipleTelemetry telemetry;
  //Stored Values
  public enum Alliance{
    BLUE, RED
  }
  public static Alliance alliance = Alliance.BLUE; //0 = blue, 1 = red
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
      outtake = new ContinuousMotorMechanism(outtakeMotor,
              360.0/28.0, 36000.0
      );
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(5);
      limelight.start();

      LLResult llResult;
      llResult = limelight.getLatestResult();
      if(llResult != null && llResult.isValid()){
        alliance = llResult.getFiducialResults().get(0).getFiducialId() == 24 ? Alliance.RED : Alliance.BLUE;
      }

      telemetry.addLine("Robot successfully initialized");
      telemetry.addLine("Detected as " + (alliance == Alliance.RED ? "RED" : "BLUE") + " alliance");
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

      drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
      drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
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
    double turretXOffset = 4.67*Math.cos(Math.toRadians(-164.475)+robotPose.heading.log());
    double turretYOffset = 4.67*Math.sin(Math.toRadians(-164.475)+robotPose.heading.log());
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

  public static double shootOuttake(Pose2d robotPose){
    double turretXOffset = 4.67*Math.cos(Math.toRadians(-164.475)+robotPose.heading.log());
    double turretYOffset = 4.67*Math.sin(Math.toRadians(-164.475)+robotPose.heading.log());
    double currDistance = Math.sqrt(
            Math.pow((alliance == Alliance.BLUE ? -72.0 : 72.0)-robotPose.position.y-turretYOffset, 2)+
            Math.pow(-72.0-robotPose.position.x-turretXOffset, 2)
    );
    telemetry.addData("Curr Distance (ft)", currDistance);
    double targetArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> 50.0/12.0 < artifactPos(vel, 45.0, currDistance/12.0));
    telemetry.addData("Target Artifact Vel (ft/s)", targetArtifactVel);
    double targetOuttakeVel = 1.4*(targetArtifactVel/0.8);
    telemetry.addData("Target Outtake Vel (ft/s)", targetOuttakeVel);
    double targetOuttakeAngVel = targetOuttakeVel/(2*Math.PI*0.1181102362)*360.0;
    telemetry.addData("Target Outtake Ang Vel (deg/s)", targetOuttakeAngVel);
    double targetOuttakeAngVelInitial = targetOuttakeAngVel/0.740740741;
    telemetry.addData("Target Outtake Ang Vel Initial (deg/s)", targetOuttakeAngVelInitial);
    telemetry.addData("Actual Outtake Ang Vel (deg/s)", outtake.getVel());
    outtake.setPos(0, targetOuttakeAngVelInitial);

    double minArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> 40.0/12.0 < artifactPos(vel, 45.0, currDistance/12.0));
    telemetry.addData("Min Artifact Vel (ft/s)", minArtifactVel);
    double minOuttakeVel = 1.4*(minArtifactVel/0.8);
    telemetry.addData("Min Outtake Vel (ft/s)", minOuttakeVel);
    double minOuttakeAngVel = minOuttakeVel/(2*Math.PI*0.1181102362)*360.0;
    telemetry.addData("Min Outtake Ang Vel (deg/s)", minOuttakeAngVel);
    double minOuttakeAngVelInitial = minOuttakeAngVel/0.740740741;
    telemetry.addData("Min Outtake Ang Vel Initial (deg/s)", minOuttakeAngVelInitial);
    return minOuttakeAngVelInitial;
  }

  public static double shootOuttake(){
    Pose2d pose = drive.localizer.getPose();
    return shootOuttake(pose);
  }

  public static void shootSequence(){
    aimOuttakeTurret();
    double minOuttakeVel = shootOuttake();
    boolean readyToShoot = true;
    int shots = 0;
    while(shots < 3){
      if(outtake.getVel() >= minOuttakeVel){
        transfer.motor.setPower(1.0);
        readyToShoot = true;
      }
      else{
        transfer.motor.setPower(0.0);
        if(readyToShoot){
          shots++;
        }
        readyToShoot = false;
      }
    }
  }
}