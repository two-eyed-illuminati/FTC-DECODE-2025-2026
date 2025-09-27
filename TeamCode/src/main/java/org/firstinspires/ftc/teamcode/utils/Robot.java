package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;

import java.util.List;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class Robot{
  //Constants

  //Mechanisms, IMU, etc.
  public static MecanumDrive drive;
  public static MultipleTelemetry telemetry;
  public static Limelight3A limelight;
  public static int obeliskAprilTagID; // -1 means no tag detected
  public static boolean initialized = false;

  public static void initialize(HardwareMap hardwareMap, Telemetry dsTelemetry){
    drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    telemetry = new MultipleTelemetry(
            dsTelemetry, // Driver Station telemetry
            FtcDashboard.getInstance().getTelemetry() // Dashboard telemetry
    );
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(4); // Obelisk AprilTag pipeline
    limelight.start();
    obeliskAprilTagID = -1;

    initialized = true;
  }

  public static void initializeLoop(){
    LLResult result = limelight.getLatestResult();
    if(result.isValid()){
      List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
      for (LLResultTypes.FiducialResult fr : fiducialResults) {
        obeliskAprilTagID = fr.getFiducialId();
      }
      telemetry.addData("Obelisk AprilTag ID: ", obeliskAprilTagID);
      telemetry.update();
    }
  }
}