package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
import org.firstinspires.ftc.teamcode.utils.rr.MecanumDrive;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class Robot{
  //Constants
  public static double START_X = -49.0;
  public static double START_Y = -50.5;
  public static double START_HEADING = -126.5;
  public static double STOPPER_CLOSED_POS = 0.65;
  public static double STOPPER_OPEN_POS = 0.3;
  public static double FRONT_DISTANCE_SENSOR_DETECTION_THRESH = 8.2;
  public static double TURRET_OFFSET_LENGTH = 2.9;
  public static double TURRET_OFFSET_ANGLE = -180.0;
  public static double SHOOT_LEAD_TIME = 0.6;
  public static double SHOOT_MIN_HEIGHT = 40.0;
  public static double SHOOT_MAX_HEIGHT = 49.0;
  public static double SHOOT_TARGET_HEIGHT = 46.5;
  public static double SHOOT_SLOWDOWN_FACTOR = 0.8;
  public static double SHOOT_RADIUS = 0.1181102362;
  public static double SHOOT_TRANSFER_FACTOR = 0.5714;
  public static double SHOOT_GRAVITY = -30.183727034;
  public static double SHOOT_DRAG = -3.4448818898;
  public static double SHOOT_EXIT_HEIGHT = 1.25;
  public static double OUTTAKE_MAX_VEL = 30345.0;
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
      outtakeTurretMotor.setTargetPosition(0);
      outtakeTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      outtakeTurret = new MotorMechanism(outtakeTurretMotor,
              -270, 120, -384.5*270/360*4, 384.5*120/360*4, 1872);
      outtakeTurretController = new PIDFController(1.0/32.0, 0);

//      Servo hoodServo = hardwareMap.get(Servo.class, "hood");
//      hood = new ServoMechanism(hoodServo, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, 0.0, 1.0, HOOD_VEL);

      DcMotorEx outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtake");
      DcMotorEx outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtake2");
      outtakeMotors = new DualMotor(outtakeMotor1, outtakeMotor2);
      outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
      outtake = new ContinuousMotorMechanism(outtakeMotors,
              360.0/28.0, 36000.0
      );
      outtakeController = new PIDFController(1.0/2000.0, 1.0/30345.0);

      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(5);
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
      outtakeTurret.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    intake.setPower(1.0);
    stopper.setPosition(STOPPER_CLOSED_POS);
  }

  public static double voltageToDistance(double voltage){
    return (voltage / 3.3) * 39.37;
  }

  public static Vector2d calculateGoalRelativeToOuttake(Pose2d robotPose){
    double turretXOffset = TURRET_OFFSET_LENGTH*Math.cos(Math.toRadians(TURRET_OFFSET_ANGLE)+robotPose.heading.log());
    double turretYOffset = TURRET_OFFSET_LENGTH*Math.sin(Math.toRadians(TURRET_OFFSET_ANGLE)+robotPose.heading.log());

    return new Vector2d(-71.0-robotPose.position.x-turretXOffset, (alliance == Alliance.BLUE ? -71.0 : 71.0)-robotPose.position.y-turretYOffset);
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
    double targetArtifactVel = BinarySearch.binarySearch(0.0, 1000.0,
            (vel) -> height/12.0 < calculateArtifactPos(robotPose, robotVelocity, vel,theta, currDistance/12.0));
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
      if(timeToShoot < bestTime){
          bestTime = timeToShoot;
          bestMag = mag;
          bestHoodTheta = hoodTheta;
      }
    }
    double theta = calculateOuttakeTurretAim(robotPose, robotVelocity, bestMag);
    return new double[]{theta, bestMag, bestHoodTheta};
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
      double targetPower = outtakeTurretController.getPower(outtakeTurret.getPos(), angle);
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
    Pose2d futureRobotPose = new Pose2d(robotPose.position.x + SHOOT_LEAD_TIME*robotVelocity.linearVel.x, robotPose.position.y + SHOOT_LEAD_TIME*robotVelocity.linearVel.y, robotPose.heading.toDouble());
    double theta = calculateShoot(futureRobotPose, robotVelocity, SHOOT_TARGET_HEIGHT, false)[0];
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
      outtake.motor.setPower(targetPower);
    }
    else{
      double targetPower = outtakeController.getPower(mag, mag);
      telemetry.addData("Target Outtake Power", targetPower);
      outtake.motor.setPower(targetPower);
    }
  }

  public static double[] shootOuttake(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid, double targetHeight, boolean findBestHoodAngle){
    Pose2d futureRobotPose = new Pose2d(robotPose.position.x + SHOOT_LEAD_TIME*robotVelocity.linearVel.x, robotPose.position.y + SHOOT_LEAD_TIME*robotVelocity.linearVel.y, robotPose.heading.toDouble());

    double maxMag = calculateShoot(robotPose, robotVelocity, SHOOT_MAX_HEIGHT, findBestHoodAngle)[1];
    double maxOuttakeAngVelInitial = calculateOuttakeVelInitial(maxMag);
    telemetry.addData("Max Outtake Ang Vel Initial (deg/s)", maxOuttakeAngVelInitial);

    double minMag = calculateShoot(robotPose, robotVelocity, SHOOT_MIN_HEIGHT, findBestHoodAngle)[1];
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

    return new double[]{minOuttakeAngVelInitial, maxOuttakeAngVelInitial};
  }
  // For Auto
  public static double[] shootOuttake(Pose2d robotPose, double targetHeight){
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    return shootOuttake(robotPose, robotVelocity, false, targetHeight, false);
  }
  // For Tele
  public static double[] shootOuttake(PoseVelocity2d robotVelocity, boolean findBestHoodAngle){
    Pose2d robotPose = drive.localizer.getPose();
    return shootOuttake(robotPose, robotVelocity, true, SHOOT_TARGET_HEIGHT, findBestHoodAngle);
  }
  public static double[] shootOuttake(){
    Pose2d robotPose = drive.localizer.getPose();
    PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    return shootOuttake(robotPose, robotVelocity, true, SHOOT_TARGET_HEIGHT, true);
  }
  public static class AimOuttakeTurretAction implements Action {
    Pose2d endPose;
    public AimOuttakeTurretAction(Pose2d endPose){
      this.endPose = endPose;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      aimOuttakeTurret(endPose);
      packet.put("Outtake Turret Pos", outtakeTurret.getPos());
      packet.put("Outtake Turret Error", Math.abs(outtakeTurret.getPos()-(calculateOuttakeTurretAim(endPose)-Math.toDegrees(endPose.heading.toDouble()))));
      if(Math.abs(outtakeTurret.getPos()-(calculateOuttakeTurretAim(endPose)-Math.toDegrees(endPose.heading.toDouble()))) < 5.0
              && Math.abs(outtakeTurret.getVel()) < 10.0
      ){
        outtakeTurret.motor.setPower(0);
        return false;
      }
      return true;
    }
  }
  public static Action getAimOuttakeTurretAction(Pose2d endPose){
    return new AimOuttakeTurretAction(endPose);
  }

  public static class ShootSequenceAction implements Action {
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime elapsedSinceTimeStartAttemptToShoot = new ElapsedTime();
    boolean attemptingToShoot = false;
    boolean started = false;
    double time = 2.25;
    public ShootSequenceAction(){
      time = 2.25;
    }
    public ShootSequenceAction(double time){
      this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
      if(!started) {
        elapsedTime.reset();
        elapsedSinceTimeStartAttemptToShoot.reset();
        started = true;
      }

      if(elapsedTime.seconds() > time){
        intake.setPower(0);
        outtake.setPos(0, 0);
        return false;
      }
      stopper.setPosition(STOPPER_OPEN_POS);
      aimOuttakeTurret();
      double[] outtakeVels = shootOuttake();

      if(elapsedTime.seconds() < 0.2){
        attemptingToShoot = false;
      }
      else if(((outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1]) ||
              elapsedTime.seconds() > time - 0.2) &&
              elapsedSinceTimeStartAttemptToShoot.seconds() < 0.6
      ){
        if(!attemptingToShoot){
          elapsedSinceTimeStartAttemptToShoot.reset();
        }
        attemptingToShoot = true;
        intake.setPower(1.0);
      }
      else if(elapsedSinceTimeStartAttemptToShoot.seconds() % 1.0 < 0.175){
        attemptingToShoot = false;
        intake.setPower(-1.0);
      }
      else{
        attemptingToShoot = false;
        intake.setPower((outtake.getVel() >= outtakeVels[0] && outtake.getVel() <= outtakeVels[1]) ? 1.0 : 0.0);
      }

      packet.put("Elapsed Time (s)", elapsedTime.seconds());
      packet.put("Outtake Vel (deg/s)", outtake.getVel());
      packet.put("Min Outtake Vel (deg/s)", outtakeVels[0]);
      packet.put("Max Outtake Vel (deg/s)", outtakeVels[1]);
      packet.put("Outtake Turret Pos (deg)", outtakeTurret.getPos());

      return true;
    }
  }
  public static Action getShootSequenceAction(){
    return new ShootSequenceAction(); // Put this in a function so that it's easier to convert AutoBuilder to MeepMeep
  }
}
