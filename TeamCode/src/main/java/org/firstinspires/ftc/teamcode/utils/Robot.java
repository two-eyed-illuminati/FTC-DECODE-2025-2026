package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
  public static DcMotorEx transfer;
  public static MotorMechanism outtakeTurret;
  public static DcMotorEx outtake;
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
      transfer = hardwareMap.get(DcMotorEx.class, "transfer");
      DcMotorEx outtakeTurretMotor = hardwareMap.get(DcMotorEx.class, "outtakeTurret");
      outtakeTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      outtakeTurretMotor.setTargetPosition(0);
      outtakeTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      outtakeTurret = new MotorMechanism(outtakeTurretMotor,
              -90, 210, -537.7/4*4, 537.7/4*4, 1872);
      outtake = hardwareMap.get(DcMotorEx.class, "outtake");
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
}