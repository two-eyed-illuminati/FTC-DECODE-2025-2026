package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.utils.rr.Drawing;
import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.rr.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.utils.rr.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.utils.rr.messages.PoseMessage;

import java.util.List;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class Robot{
  //Constants
  public static double START_X = -49.0;
  public static double START_Y = -50.5;
  public static double START_HEADING = -126.5;
  public static double STOPPER_CLOSED_POS = 0.85;
  public static double STOPPER_OPEN_POS = 0.228;
  public static double FRONT_DISTANCE_SENSOR_DETECTION_THRESH = 7.5;
  public static double TOP_DISTANCE_SENSOR_DETECTION_THRESH = 5.0;
  public static double TURRET_OFFSET_LENGTH = 2.9;
  public static double TURRET_OFFSET_ANGLE = -180.0;
  public static double SHOOT_TURRET_LEAD_TIME_CLOSE = 0.42;
  public static double SHOOT_TURRET_LEAD_TIME_FAR = 0.35;
  public static boolean leadEnabled = true;
  public static double SHOOT_OUTTAKE_LEAD_TIME_CLOSE = 0.3;
  public static double SHOOT_OUTTAKE_LEAD_TIME_FAR = 0.15;
  public static double SHOOT_MIN_HEIGHT_CLOSE = 40.0;
  public static double SHOOT_MIN_HEIGHT_FAR = 47.5;
  public static double SHOOT_MAX_HEIGHT_CLOSE = 47.0;
  public static double SHOOT_MAX_HEIGHT_FAR = 51.0;
  public static double SHOOT_TARGET_HEIGHT_CLOSE = 45.0;
  public static double SHOOT_TARGET_HEIGHT_FAR = 48.5;
  public static double SHOOT_SLOWDOWN_FACTOR = 0.83;
  public static double SHOOT_RADIUS = 0.1181102362;
  public static double SHOOT_TRANSFER_FACTOR = 0.5714;
  public static double SHOOT_GRAVITY = -30.183727034;
  public static double SHOOT_DRAG = -3.4448818898;
  public static double SHOOT_EXIT_HEIGHT = 1.25;
  public static double OUTTAKE_MAX_VEL = 28200.0;
  public static double OUTTAKE_C_COEFF = 1.12;
  public static double HOOD_MIN_ANGLE = 45.0;
  public static double HOOD_MAX_ANGLE = 45.0;
  public static double HOOD_VEL = 0.5;
  //Mechanisms, IMU, etc.
  public static MecanumDrive drive;
  public static DcMotorEx intake;
  public static Servo stopper;
  public static MotorMechanism outtakeTurret;
  public static ServoMechanism hood;
  public static PIDFController outtakeTurretController;
  public static PIDFController outtakeTurretVelocityController;
  public static DualMotor outtakeMotors;
  public static ContinuousMotorMechanism outtake;
  public static PIDFController outtakeController;
  public static Limelight3A limelight;
  public static AnalogInput frontDistanceSensor;
  public static AnalogInput topDistanceSensor;
  public static Servo ledLeft;
  public static Servo ledRight;
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
      intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      stopper = hardwareMap.get(Servo.class, "stopper");

      DcMotorEx outtakeTurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
      outtakeTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      outtakeTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      outtakeTurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      outtakeTurret = new MotorMechanism(outtakeTurretMotor,
              -180, 132, -384.5*180/360*4, 384.5*132/360*4, 1872);
      outtakeTurretController = new PIDFController(1/36.0, 1/480.0,0);

//      Servo hoodServo = hardwareMap.get(Servo.class, "hood");
//      hood = new ServoMechanism(hoodServo, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, 0.0, 1.0, HOOD_VEL);

      DcMotorEx outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtake");
      DcMotorEx outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtake2");
      outtakeMotors = new DualMotor(outtakeMotor1, outtakeMotor2);
      outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
      outtake = new ContinuousMotorMechanism(outtakeMotors,
              360.0/28.0, OUTTAKE_MAX_VEL
      );
      outtakeController = new PIDFController(1.0/2000.0, 0,12.4/(OUTTAKE_MAX_VEL));

      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.setPollRateHz(100);
      limelight.pipelineSwitch(0);
      limelight.start();

      frontDistanceSensor = hardwareMap.get(AnalogInput.class, "frontDistance");
      topDistanceSensor = hardwareMap.get(AnalogInput.class, "topDistance");

      ledLeft = hardwareMap.get(Servo.class, "ledLeft");
      ledRight = hardwareMap.get(Servo.class, "ledRight");

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
      intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      outtakeTurret.motor.setTargetPosition(outtakeTurret.motor.getCurrentPosition());
      outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      outtakeTurret.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      outtakeMotors.motor2.setDirection(DcMotorSimple.Direction.REVERSE);

      drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
      drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

      drive.localizer.setPose(pose);
    }
  }

  public static void beginIntake(){
    intake.setPower(0.75);
    stopper.setPosition(STOPPER_CLOSED_POS);
  }
  public static void stopIntake(){
    intake.setPower(0.75);
    stopper.setPosition(STOPPER_CLOSED_POS);
  }
  public static void openStopper(){
    stopper.setPosition(STOPPER_OPEN_POS);
  }

  public static double voltageToDistance(double voltage){
    return (voltage / 3.3) * 39.37;
  }

  public static Vector2d calculateGoalRelativeToOuttake(Pose2d robotPose){
    double turretXOffset = TURRET_OFFSET_LENGTH*Math.cos(Math.toRadians(TURRET_OFFSET_ANGLE)+robotPose.heading.log());
    double turretYOffset = TURRET_OFFSET_LENGTH*Math.sin(Math.toRadians(TURRET_OFFSET_ANGLE)+robotPose.heading.log());

    return new Vector2d(-70.0-robotPose.position.x-turretXOffset, (alliance == Alliance.BLUE ? -70.0 : 70.0)-robotPose.position.y-turretYOffset);
  }
  public static double calculateOuttakeTurretAim(Pose2d robotPose){
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double targetTurretPos = Math.toDegrees(
            Math.atan2(
                    goalRelativeToOuttake.y,
                    goalRelativeToOuttake.x
            )
    );
    return targetTurretPos;
  }
  public static double calculateOuttakeTurretAim(Pose2d robotPose, PoseVelocity2d robotVel, double shootVel){
    double theta = Math.toRadians(calculateOuttakeTurretAim(robotPose));
    Vector2d robotLinearVelGoalPerspective = Rotation2d.fromDouble(-theta).times(robotVel.linearVel);
    Rotation2d velAdjustedAngle = Rotation2d.fromDouble(Math.asin(-(robotLinearVelGoalPerspective.y/12.0)/shootVel));
    double robotPovAngle = Rotation2d.fromDouble(theta).times(velAdjustedAngle).log();
    return Math.toDegrees(robotPovAngle);
  }
  public static double calculateArtifactPos(Pose2d robotPose, PoseVelocity2d robotVel, double v0, double theta, double x){
    double angleToGoal = Math.toRadians(calculateOuttakeTurretAim(robotPose));
    Vector2d robotLinearVelGoalPerspective = Rotation2d.fromDouble(-angleToGoal).times(robotVel.linearVel);
    Rotation2d velAdjustedAngle = Rotation2d.fromDouble(Math.asin(-(robotLinearVelGoalPerspective.y/12.0)/v0));

    double v0x = v0*Math.sin(Math.toRadians(theta))*Math.cos(velAdjustedAngle.log()) + robotLinearVelGoalPerspective.x/12.0;
    double v0y = v0*Math.cos(Math.toRadians(theta));
    double a = 0.5*(SHOOT_DRAG);
    double b = v0x;
    double t = (-b+Math.sqrt(b*b-4*a*(-x)))/(2*a);
    double y = SHOOT_EXIT_HEIGHT+v0y*t+0.5*(SHOOT_GRAVITY)*t*t;

    return y;
  }
  public static double calculateArtifactShootVel(Pose2d robotPose, PoseVelocity2d robotVelocity, double theta, double height){
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
            goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    telemetry.addData("Curr Distance (in)", currDistance);
    double targetArtifactVel = BinarySearch.binarySearch(0.0, 100.0,
            (vel) -> height/12.0 < calculateArtifactPos(robotPose, robotVelocity, vel, theta, currDistance/12.0));
    telemetry.addData("Target Artifact Vel (ft/s)", targetArtifactVel);
    return targetArtifactVel;
  }
  public static double calculateOuttakeVelInitial(double targetArtifactVel){
    return (((targetArtifactVel/SHOOT_TRANSFER_FACTOR)/(2*Math.PI*SHOOT_RADIUS))*360.0)/SHOOT_SLOWDOWN_FACTOR;
  }
  public static double calculateTimeToChangeOuttakeVel(double initialVel, double targetVel){
    if(targetVel > OUTTAKE_MAX_VEL || targetVel < -OUTTAKE_MAX_VEL){
      return Double.POSITIVE_INFINITY;
    }
    double maxVel = targetVel > initialVel ? OUTTAKE_MAX_VEL : -OUTTAKE_MAX_VEL;
    return Math.log((-targetVel+maxVel)/(-initialVel+maxVel))/(-OUTTAKE_C_COEFF);
  }
  public static double calculateTimeToChangeHoodAngle(double initialAngle, double targetAngle){
    return Math.abs(targetAngle-initialAngle)/(HOOD_VEL*360.0);
  }
  public static double[] calculateShoot(Pose2d robotPose, PoseVelocity2d robotVelocity, double height, boolean findBestHoodAngle){
    double bestMag = Double.POSITIVE_INFINITY;
    double bestHoodTheta = HOOD_MIN_ANGLE;
    double bestTime = Double.POSITIVE_INFINITY;
    for(double hoodTheta = HOOD_MIN_ANGLE; hoodTheta <= (findBestHoodAngle ? HOOD_MAX_ANGLE : HOOD_MIN_ANGLE); hoodTheta += 0.5){
      double mag = calculateArtifactShootVel(robotPose, robotVelocity, hoodTheta, height);
      double timeToVel = calculateTimeToChangeOuttakeVel(outtake.getVel(), calculateOuttakeVelInitial(mag));
//      double timeToAngle = calculateTimeToChangeHoodAngle(hood.getPos(), hoodTheta);
      double timeToAngle = 0.0;
      double timeToShoot = Math.max(timeToVel, timeToAngle);
      if(timeToShoot < bestTime || bestTime == Double.POSITIVE_INFINITY){
          bestTime = timeToShoot;
          bestMag = mag;
          bestHoodTheta = hoodTheta;
      }
    }
    double theta = calculateOuttakeTurretAim(robotPose, robotVelocity, bestMag);
    return new double[]{theta, bestMag, bestHoodTheta};
  }

  public static double turretLeadTime(Pose2d robotPose){
    if(!leadEnabled){
      return 0;
    }
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
                    goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    if(currDistance > 120.0 || robotPose.position.x > 24.0){
      return SHOOT_TURRET_LEAD_TIME_FAR;
    }
    else{
      return SHOOT_TURRET_LEAD_TIME_CLOSE;
    }
  }
  public static double outtakeLeadTime(Pose2d robotPose){
    if(!leadEnabled){
      return 0;
    }
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
                    goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    if(currDistance > 120.0 || robotPose.position.x > 24.0){
      return SHOOT_OUTTAKE_LEAD_TIME_FAR;
    }
    else{
      return SHOOT_OUTTAKE_LEAD_TIME_CLOSE;
    }
  }
  public static double shootMinHeight(Pose2d robotPose){
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
                    goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    if(currDistance > 120.0 || robotPose.position.x > 24.0){
      return SHOOT_MIN_HEIGHT_FAR;
    }
    else{
      return SHOOT_MIN_HEIGHT_CLOSE;
    }
  }
  public static double shootMaxHeight(Pose2d robotPose){
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
                    goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    if(currDistance > 120.0 || robotPose.position.x > 24.0){
      return SHOOT_MAX_HEIGHT_FAR;
    }
    else{
      return SHOOT_MAX_HEIGHT_CLOSE;
    }
  }
  public static double shootTargetHeight(Pose2d robotPose){
    Vector2d goalRelativeToOuttake = calculateGoalRelativeToOuttake(robotPose);
    double currDistance = Math.sqrt(
            goalRelativeToOuttake.x*goalRelativeToOuttake.x+
                    goalRelativeToOuttake.y*goalRelativeToOuttake.y
    );
    if(currDistance > 120.0 || robotPose.position.x > 24.0){
      return SHOOT_TARGET_HEIGHT_FAR;
    }
    else{
      return SHOOT_TARGET_HEIGHT_CLOSE;
    }
  }

  public static void aimOuttakeTurret(double theta, Pose2d robotPose, boolean pid){
    double angle = (theta-Math.toDegrees(robotPose.heading.toDouble())) % 360.0;
    if(angle > 180.0){
      angle -= 360.0;
    }
    if(angle < -180.0){
      angle += 360.0;
    }
    telemetry.addData("Target Angle", angle);
    if(Math.abs(Clamp.clamp(angle, outtakeTurret.minPos, outtakeTurret.maxPos)-angle) > 20){
      angle = outtakeTurret.getPos();
    }
    telemetry.addData("Target Angle After Unnecessary Motion Reduction", Clamp.clamp(angle, outtakeTurret.minPos, outtakeTurret.maxPos));
    if(pid){
      angle = Clamp.clamp(angle, outtakeTurret.minPos, outtakeTurret.maxPos);
      double oldP = outtakeTurretController.pCoefficient;
      if(Math.abs(outtakeTurret.getPos() - angle) < 2.0){
        outtakeTurretController.pCoefficient = 1/36.0;
      }
      if(oldP > 1/16.0 && Math.abs(outtakeTurret.getPos() - angle) < 4.0){
        outtakeTurretController.pCoefficient = 1/6.0;
      }
      double targetPower = outtakeTurretController.getPower(outtakeTurret.getPos(), angle);
      if(Math.abs(outtakeTurret.getPos() - angle) > 0.5 && oldP <= 1/16.0){
        targetPower = Math.signum(targetPower)*(Math.abs(targetPower)+0.05);
      }
      outtakeTurretController.pCoefficient = oldP;
      telemetry.addData("Target Outtake Turret Power", targetPower);
      outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      outtakeTurret.motor.setPower(targetPower);
    }
    else {
      outtakeTurret.setPos(angle);
      outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    telemetry.addData("Outtake Turret Pos", outtakeTurret.getPos());
  }
  public static void aimOuttakeTurret(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid){
    Pose2d futureRobotPose = new Pose2d(robotPose.position.x + turretLeadTime(robotPose)*robotVelocity.linearVel.x, robotPose.position.y + turretLeadTime(robotPose)*robotVelocity.linearVel.y, robotPose.heading.toDouble());
    double theta = calculateShoot(futureRobotPose, robotVelocity, shootTargetHeight(robotPose), false)[0];
    aimOuttakeTurret(theta, futureRobotPose, pid);
  }
  // For Auto
  public static void aimOuttakeTurret(Pose2d robotPose){
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    aimOuttakeTurret(robotPose, robotVelocity, true);
  }
  // For Tele
  public static void aimOuttakeTurret(PoseVelocity2d robotVelocity){
    Pose2d robotPose = drive.localizer.getPose();
    aimOuttakeTurret(robotPose, robotVelocity, true);
  }
  public static void aimOuttakeTurret(){
    Pose2d robotPose = drive.localizer.getPose();
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    aimOuttakeTurret(robotPose, robotVelocity, true);
  }

  public static void shootOuttake(double mag, boolean pid){
    if(pid) {
      double targetPower = outtakeController.getPower(outtake.getVel(), mag);
      telemetry.addData("Target Outtake Power", targetPower);
      telemetry.addData("Voltage", drive.voltageSensor.getVoltage());
      outtake.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      outtake.motor.setPower(targetPower);
    }
    else{
      double targetPower = outtakeController.getPower(mag, mag);
      telemetry.addData("Target Outtake Power", targetPower);
      telemetry.addData("Voltage", drive.voltageSensor.getVoltage());
      outtake.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      outtake.motor.setPower(targetPower);
    }
  }

  public static double[] shootOuttake(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid, double targetHeight, boolean findBestHoodAngle){
    Pose2d futureRobotPose = new Pose2d(robotPose.position.x + outtakeLeadTime(robotPose)*robotVelocity.linearVel.x, robotPose.position.y + outtakeLeadTime(robotPose)*robotVelocity.linearVel.y, robotPose.heading.toDouble());

    double maxMag = calculateShoot(robotPose, robotVelocity, shootMaxHeight(robotPose), findBestHoodAngle)[1];
    double maxOuttakeAngVelInitial = calculateOuttakeVelInitial(maxMag);
    telemetry.addData("Max Outtake Ang Vel Initial (deg/s)", maxOuttakeAngVelInitial);

    double minMag = calculateShoot(robotPose, robotVelocity, shootMinHeight(robotPose), findBestHoodAngle)[1];
    double minOuttakeAngVelInitial = calculateOuttakeVelInitial(minMag);
    telemetry.addData("Min Outtake Ang Vel Initial (deg/s)", minOuttakeAngVelInitial);

    double[] targetShootParams = calculateShoot(futureRobotPose, robotVelocity, targetHeight, findBestHoodAngle);
    double targetMag = targetShootParams[1];
    double targetHoodTheta = targetShootParams[2];
    double targetOuttakeAngVelInitial = calculateOuttakeVelInitial(targetMag);
    telemetry.addData("Target Outtake Ang Vel Initial (deg/s)", targetOuttakeAngVelInitial);

    shootOuttake(targetOuttakeAngVelInitial, pid);
//    hood.setPos(targetHoodTheta);
    telemetry.addData("Actual Outtake Ang Vel (deg/s)", outtake.getVel());

    return new double[]{minOuttakeAngVelInitial, maxOuttakeAngVelInitial, targetOuttakeAngVelInitial};
  }
  // For Auto
  public static double[] shootOuttake(Pose2d robotPose, double targetHeight){
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    return shootOuttake(robotPose, robotVelocity, true, targetHeight, false);
  }
  // For Tele
  public static double[] shootOuttake(PoseVelocity2d robotVelocity, boolean findBestHoodAngle){
    Pose2d robotPose = drive.localizer.getPose();
    return shootOuttake(robotPose, robotVelocity, true, shootTargetHeight(robotPose), findBestHoodAngle);
  }
  public static double[] shootOuttake(){
    Pose2d robotPose = drive.localizer.getPose();
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    return shootOuttake(robotPose, robotVelocity, true, shootTargetHeight(robotPose), true);
  }

  public static class ReverseIntakeAction implements Action {
    ElapsedTime elapsedTime = new ElapsedTime();
    boolean started = false;
    double time = 0.5;
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      if(!started){
        started = true;
        elapsedTime.reset();
      }

      if(elapsedTime.seconds() > time){
        stopIntake();
        return false;
      }
      else{
        intake.setPower(-1.0);
        return true;
      }
    }
  }
  public static Action getReverseIntakeAction(){
    return new ReverseIntakeAction();
  }

  public static boolean STOP_AIM_TURRET_ACTION = false;
  public static class AimOuttakeTurretAction implements Action {
    Pose2d endPose;
    public AimOuttakeTurretAction(Pose2d endPose){
      this.endPose = endPose;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      aimOuttakeTurret(endPose);
      double targetTurretPos = calculateOuttakeTurretAim(endPose)-Math.toDegrees(endPose.heading.toDouble());
      packet.put("Outtake Turret Pos", outtakeTurret.getPos());
      packet.put("Outtake Turret Error", Math.abs(outtakeTurret.getPos()-targetTurretPos));
      if(STOP_AIM_TURRET_ACTION){
        STOP_AIM_TURRET_ACTION = false;
        return false;
      }
      return true;
    }
  }
  public static Action getAimOuttakeTurretAction(Pose2d endPose){
    return new AimOuttakeTurretAction(endPose);
  }

  public static boolean STOP_SHOOT_OUTTAKE_ACTION = false;
  public static class ShootOuttakeAction implements Action {
    Pose2d endPose;
    public ShootOuttakeAction(Pose2d endPose){this.endPose = endPose;}

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      double[] outtakeVels = shootOuttake(endPose, shootTargetHeight(endPose));
      packet.put("Outtake Vel (deg/s)", outtake.getVel());
      packet.put("Min Outtake Vel (deg/s)", outtakeVels[0]);
      packet.put("Max Outtake Vel (deg/s)", outtakeVels[1]);
      packet.put("Target Outtake Vel (deg/s)", outtakeVels[2]);
      if(STOP_SHOOT_OUTTAKE_ACTION){
        STOP_SHOOT_OUTTAKE_ACTION = false;
        return false;
      }
      return true;
    }
  }
  public static Action getShootOuttakeAction(Pose2d endPose){
    return new ShootOuttakeAction(endPose);
  }

  public static class ShootSequenceAction implements Action {
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedSinceTimeStartAttemptToShoot = new ElapsedTime();
    ElapsedTime elapsedTimeSinceBallDetected = new ElapsedTime();
    boolean attemptingToShoot = false;
    boolean started = false;
    boolean spunUp = false;
    double MAX_TIME = 3.0;
    double time;
    public ShootSequenceAction(){
      time = 1.0;
    }
    public ShootSequenceAction(double time){
      this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      aimOuttakeTurret();
      double[] outtakeVels = shootOuttake();
      drive.updatePoseEstimate();

      if(!spunUp && (outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1])){
        spunUp = true;
      }
      else if(!spunUp){
        return true;
      }

      if(!started) {
        elapsedTime.reset();
        elapsedSinceTimeStartAttemptToShoot.reset();
        started = true;
      }

      telemetry.addData("Stopper Pos", stopper.getPosition());

      double topDistance = voltageToDistance(topDistanceSensor.getVoltage());
      if(topDistance < TOP_DISTANCE_SENSOR_DETECTION_THRESH){
        elapsedTimeSinceBallDetected.reset();
      }
      if(elapsedTime.seconds() > MAX_TIME ||
        (elapsedTimeSinceBallDetected.seconds() > 0.2 && elapsedTime.seconds() > time)){
        intake.setPower(0);
        stopper.setPosition(STOPPER_CLOSED_POS);
        return false;
      }

      if(((outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1]) ||
              elapsedTime.seconds() > MAX_TIME - 0.2) &&
              elapsedSinceTimeStartAttemptToShoot.seconds() < 0.6
      ){
        if(!attemptingToShoot){
          elapsedSinceTimeStartAttemptToShoot.reset();
        }
        stopper.setPosition(Robot.STOPPER_OPEN_POS);
        attemptingToShoot = true;
        intake.setPower(1.0);
      }
      else if(elapsedSinceTimeStartAttemptToShoot.seconds() % 1.0 < 0.175 && elapsedTime.seconds() > time){
        attemptingToShoot = false;
        intake.setPower(-1.0);
      }
      else{
        attemptingToShoot = false;
        intake.setPower(1.0);
      }

      packet.put("Elapsed Time (s)", elapsedTime.seconds());
      packet.put("Intake Power", intake.getPower());
      packet.put("Top Distance (in)", topDistance);
      packet.put("Outtake Vel (deg/s)", outtake.getVel());
      packet.put("Min Outtake Vel (deg/s)", outtakeVels[0]);
      packet.put("Max Outtake Vel (deg/s)", outtakeVels[1]);
      packet.put("Target Outtake Vel (deg/s)", outtakeVels[2]);
      packet.put("Outtake Turret Pos (deg)", outtakeTurret.getPos());
      packet.put("Outtake Power", outtake.motor.getPower());
      double targetTurretPos = calculateOuttakeTurretAim(drive.localizer.getPose())-Math.toDegrees(drive.localizer.getPose().heading.toDouble());
      packet.put("Outtake Turret Pos", outtakeTurret.getPos());
      packet.put("Outtake Turret Error", Math.abs(outtakeTurret.getPos()-targetTurretPos));

      return true;
    }
  }
  public static Action getShootSequenceAction(){
    return new ShootSequenceAction(); // Put this in a function so that it's easier to convert AutoBuilder to MeepMeep
  }
  public static Action getShootSequenceAction(double time){
    return new ShootSequenceAction(time); // Put this in a function so that it's easier to convert AutoBuilder to MeepMeep
  }

  public static class LooseIntakeAction implements Action {
    double P_VALUE = 1/35.0;
    boolean started = false;
    boolean returning = false;
    double time = 10.0;
    ElapsedTime elapsedTime;
    ElapsedTime elapsedTimeOfBallDetected;
    ElapsedTime elapsedTimeSinceLastForward;
    Pose2d endPose;
    Action goToEndPoseAction;
    public LooseIntakeAction(Pose2d endPose){
      this.endPose = endPose;
      elapsedTime = new ElapsedTime();
      elapsedTimeOfBallDetected = new ElapsedTime();
      elapsedTimeSinceLastForward = new ElapsedTime();
    }
    public boolean run(@NonNull TelemetryPacket packet){
      if(!started){
        started = true;
        elapsedTime.reset();
        elapsedTimeOfBallDetected.reset();
        elapsedTimeSinceLastForward.reset();
        beginIntake();
      }

      packet.put("Returning?", returning);

      drive.updatePoseEstimate();

      if(!returning) {
        if (voltageToDistance(frontDistanceSensor.getVoltage()) > FRONT_DISTANCE_SENSOR_DETECTION_THRESH) {
          elapsedTimeOfBallDetected.reset();
        }
        packet.put("Front Distance", voltageToDistance(frontDistanceSensor.getVoltage()));
        packet.put("Elapsed Time of Ball Detected", elapsedTimeOfBallDetected.seconds());

        limelight.updatePythonInputs(new double[]{alliance == Alliance.BLUE ? 0 : 1});
        LLResult result = Robot.limelight.getLatestResult();

        if (result != null) {
          packet.put("Artifact X", result.getTx());

          double forwardPower = -0.35;
          if (Math.abs(result.getTx()) < 20.0 && result.getTx() != 0) {
            forwardPower = 0.8;
            elapsedTimeSinceLastForward.reset();
          }
          else if (elapsedTimeSinceLastForward.seconds() < 0.3){
            forwardPower = 0.8;
          }

          double sidePower = alliance == Alliance.BLUE ? -0.5 : 0.5;
          if (result.getTx() != 0) {
            sidePower = -result.getTx() * P_VALUE;
          }

          packet.put("Forward Power", forwardPower);

          drive.setDrivePowers(
                  new PoseVelocity2d(
                          new Vector2d(
                                  forwardPower,
                                  sidePower
                          ),
                          0
                  )
          );
        } else {
          packet.addLine("Invalid Limelight Result");
          packet.put("Is Not Null?", result != null);
        }

        packet.put("Time of Ball Detected", elapsedTimeOfBallDetected);
        packet.put("X", drive.localizer.getPose().position.x);

        if (elapsedTime.seconds() > time || elapsedTimeOfBallDetected.seconds() > 0.6 || drive.localizer.getPose().position.x < 30) {
          stopIntake();
          returning = true;
          goToEndPoseAction = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(endPose.position, endPose.heading).build();
        }
      }
      else{
//        return false;
        return goToEndPoseAction.run(packet);
      }

      return true;
    }
  }
  public static Action getLooseIntakeAction(Pose2d endPose){
    return new LooseIntakeAction(endPose);
  }
  public static class CorrectSecondsAction implements Action {
    public Pose2d endPose;
    private double beginTs = -1;
    public double numSeconds;

    public CorrectSecondsAction(Pose2d endPose, double numSeconds) {
      this.endPose = endPose;
      this.numSeconds = numSeconds;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p){
      double t;
      if (beginTs < 0) {
        beginTs = Actions.now();
        t = 0;
      } else {
        t = Actions.now() - beginTs;
      }

      if (t >= numSeconds) {
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);

        return false;
      }

      Pose2dDual<Time> txWorldTarget = Pose2dDual.constant(endPose, 3);

      PoseVelocity2d robotVelRobot = drive.updatePoseEstimate();

      PoseVelocity2dDual<Time> command = new HolonomicController(
              MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.headingGain,
              MecanumDrive.PARAMS.axialVelGain, MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain
      )
              .compute(txWorldTarget, drive.localizer.getPose(), robotVelRobot);

      MecanumKinematics.WheelVelocities<Time> wheelVels = drive.kinematics.inverse(command);
      double voltage = drive.voltageSensor.getVoltage();

      final MotorFeedforward feedforward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
              MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick, MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);
      double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
      double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
      double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
      double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

      drive.leftFront.setPower(leftFrontPower);
      drive.leftBack.setPower(leftBackPower);
      drive.rightBack.setPower(rightBackPower);
      drive.rightFront.setPower(rightFrontPower);

      p.put("flPower", leftFrontPower);
      p.put("blPower", leftBackPower);
      p.put("brPower", rightBackPower);
      p.put("frPower", rightFrontPower);

      p.put("x", drive.localizer.getPose().position.x);
      p.put("y", drive.localizer.getPose().position.y);
      p.put("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

      Pose2d error = txWorldTarget.value().minusExp(drive.localizer.getPose());
      p.put("xError", error.position.x);
      p.put("yError", error.position.y);
      p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

      p.put("xTarget", txWorldTarget.value().position.x);
      p.put("yTarget", txWorldTarget.value().position.y);
      p.put("headingTarget (deg", Math.toDegrees(error.heading.toDouble()));

      return true;
    }
  }
  public static Action getCorrectSecondsAction(Pose2d endPose, double numSeconds){
    return new CorrectSecondsAction(endPose, numSeconds);
  }
}
