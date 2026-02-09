package com.example.meepmeep;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
public class Robot{
    //Constants
    public static double START_X = -57.0586;
    public static double START_Y = -43.8964;
    public static double START_HEADING = -126.5;
    //Mechanisms, IMU, etc.
    //Stored Values
    public enum Alliance{
        BLUE, RED
    }
    public static Alliance alliance = Alliance.BLUE; //0 = blue, 1 = red
    public static Pose2d pose = new Pose2d(0, 0, 0);
    public static boolean initialized = false;

    public static void initialize(){

        initialized = true;
    }

    public static double calculateOuttakeTurretAim(Pose2d robotPose){
        double turretXOffset = 2.9*Math.cos(Math.toRadians(-150.0)+robotPose.heading.log());
        double turretYOffset = 2.9*Math.sin(Math.toRadians(-150.0)+robotPose.heading.log());
        double targetTurretPos = Math.toDegrees(
                Math.atan2(
                        (alliance == Alliance.BLUE ? -72.0 : 72.0)-robotPose.position.y-turretYOffset,
                        -72.0-robotPose.position.x-turretXOffset
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
    public static double artifactPos(Pose2d robotPose, PoseVelocity2d robotVel, double v0, double theta, double x){
        double angleToGoal = Math.toRadians(calculateOuttakeTurretAim(robotPose));
        Vector2d robotLinearVelGoalPerspective = Rotation2d.fromDouble(-angleToGoal).times(robotVel.linearVel);
        Rotation2d velAdjustedAngle = Rotation2d.fromDouble(Math.asin(-(robotLinearVelGoalPerspective.y/12.0)/v0));

        double v0x = v0*Math.cos(Math.toRadians(theta))*Math.cos(velAdjustedAngle.log()) + robotLinearVelGoalPerspective.x/12.0;
        double v0y = v0*Math.sin(Math.toRadians(theta));
        double a = 0.5*(-3.4448818898);
        double b = v0x;
        double t = (-b+Math.sqrt(b*b-4*a*(-x)))/(2*a);
        double y = 1.25+v0y*t+0.5*(-30.183727034)*t*t;

        return y;
    }
    public static double calculateArtifactShootVel(Pose2d robotPose, PoseVelocity2d robotVelocity, double height){
        return 0.0;
    }
    public static double[] calculateShoot(Pose2d robotPose, PoseVelocity2d robotVelocity, double height){
        double mag = calculateArtifactShootVel(robotPose, robotVelocity, height);
        double theta = calculateOuttakeTurretAim(robotPose, robotVelocity, mag);
        return new double[]{theta, mag};
    }

    public static void aimOuttakeTurret(double theta, Pose2d robotPose, boolean pid){
    }
    public static void aimOuttakeTurret(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid){
        Pose2d futureRobotPose = new Pose2d(robotPose.position.x + 0.6*robotVelocity.linearVel.x, robotPose.position.y + 0.6*robotVelocity.linearVel.y, robotPose.heading.toDouble());
        double theta = calculateShoot(futureRobotPose, robotVelocity, 46.5)[0];
        aimOuttakeTurret(theta, futureRobotPose, pid);
    }
    public static void aimOuttakeTurret(Pose2d robotPose, boolean pid){
    }
    public static void aimOuttakeTurret(PoseVelocity2d robotVelocity){
    }
    public static void aimOuttakeTurret(){
    }

    public static void shootOuttake(double mag, boolean pid){
    }

    public static double[] shootOuttake(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid, double targetHeight){
        return new double[]{0, 0};
    }
    public static double[] shootOuttake(Pose2d robotPose, PoseVelocity2d robotVelocity, boolean pid){
        return new double[]{0, 0};
    }
    public static double[] shootOuttake(Pose2d robotPose, boolean pid, double targetHeight){
        return new double[]{0, 0};
    }
    public static double[] shootOuttake(PoseVelocity2d robotVelocity){
        return new double[]{0, 0};
    }
    public static double[] shootOuttake(double targetHeight){
        return new double[]{0, 0};
    }
    public static double[] shootOuttake(){
        return new double[]{0, 0};
    }

    public static Action ShootSequenceAction(){
        return new SleepAction(2.25);
    }
}
