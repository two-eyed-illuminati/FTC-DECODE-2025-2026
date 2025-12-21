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
  public static CRServo transferIn;
  public static ContinuousMotorMechanism transferUp;
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
      transferIn = hardwareMap.get(CRServo.class, "transferIn");
      DcMotorEx transferUpMotor = hardwareMap.get(DcMotorEx.class, "transferUp");
      transferUp = new ContinuousMotorMechanism(transferUpMotor,
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
  }

  public static double artifactPos(double v0, double theta, double x){
    double v0x = v0*Math.cos(Math.toRadians(theta));
    double v0y = v0*Math.sin(Math.toRadians(theta));
    double a = 0.5*(-3.4448818898);
    double b = v0x;
    double t = (-b+Math.sqrt(b*b-4*a*(-x)))/(2*a);
    double y = 0.4+v0y*t+0.5*(-30.183727034)*t*t;

    return y;
  }

  public static void aimOuttakeTurret(){
    Pose2d pose = drive.localizer.getPose();
    telemetry.addData("x", pose.position.x);
    telemetry.addData("y", pose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    double targetTurretPos = Math.toDegrees(Math.atan2(72-pose.position.y+1.25, -72-pose.position.x+4.5));
    telemetry.addData("Target Abs Turret Pos", targetTurretPos);
    telemetry.addData("Target Rel Turret Pos", targetTurretPos-Math.toDegrees(pose.heading.toDouble()));
    double angle = (targetTurretPos-Math.toDegrees(pose.heading.toDouble())) % 360;
    if(angle > 180){
      angle -= 360;
    }
    telemetry.addData("Target Angle", angle);
    outtakeTurret.setPos(angle);
    telemetry.addData("Outtake Turret Pos", outtakeTurret.getPos());
  }

  public static void shootOuttake(){
    Pose2d pose = drive.localizer.getPose();
    double currDistance = Math.sqrt(Math.pow(72-pose.position.y+1.25, 2)+Math.pow(-72-pose.position.x+4.5, 2));
    telemetry.addData("Curr Distance (ft)", currDistance);
    double targetArtifactVel = BinarySearch.binarySearch(0, 1000,
            (vel) -> 40 > artifactPos(vel, 45, currDistance/12));
    telemetry.addData("Target Artifact Vel (ft/s)", targetArtifactVel);
    double targetOuttakeVel = 2.5*(targetArtifactVel/0.8);
    telemetry.addData("Target Outtake Vel (ft/s)", targetOuttakeVel);
    double targetOuttakeAngVel = targetOuttakeVel/(2*Math.PI*0.1181102362)*360;
    telemetry.addData("Target Outtake Ang Vel (deg/s)", targetOuttakeAngVel);
    double targetOuttakeAngVelInitial = targetOuttakeAngVel/0.740740741+1000;
    telemetry.addData("Target Outtake Ang Vel Initial (deg/s)", targetOuttakeAngVelInitial);
    telemetry.addData("Actual Outtake Ang Vel (deg/s)", outtake.motor.getVelocity()*(360/28));
    outtake.setPos(0, targetOuttakeAngVelInitial);
  }
}