package com.example.meepmeep;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

public class Robot{
    public static void beginIntake(){}
    public static void aimOuttakeTurret(Pose2d robotPose){
    }
    public static double[] shootOuttake(Pose2d robotPose, double targetHeight){
        return new double[]{0, 0};
    }
    public static Action getShootSequenceAction(){
        return new SleepAction(2.25);
    }
}
