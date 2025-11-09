package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
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
  public static MultipleTelemetry telemetry;
  public static DcMotorEx fl;
  public static DcMotorEx fr;
  public static DcMotorEx bl;
  public static DcMotorEx br;
  public static boolean initialized = false;

  public static void initialize(HardwareMap hardwareMap, Telemetry dsTelemetry){
    if(!initialized) {
      drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
      telemetry = new MultipleTelemetry(
              dsTelemetry, // Driver Station telemetry
              FtcDashboard.getInstance().getTelemetry() // Dashboard telemetry
      );
      fl = hardwareMap.get(DcMotorEx.class, "fl");
      fr = hardwareMap.get(DcMotorEx.class, "fr");
      bl = hardwareMap.get(DcMotorEx.class, "bl");
      br = hardwareMap.get(DcMotorEx.class, "br");
      telemetry.addLine("INFO: ROBOT SUCCESSFULLY INITIALIZED");
    }
    else{
      telemetry.addLine("INFO: ROBOT USING PREVIOUS INITIALIZATION STATE");
      telemetry.addLine("INFO: IF YOU WOULD LIKE TO UNINITIALIZE, RUN THE \"UninitializeRobot\" OP MODE");
    }

    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    initialized = true;
    telemetry.update();
  }
}