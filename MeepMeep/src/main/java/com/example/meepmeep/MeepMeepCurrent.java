package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCurrent {
    public static double START_X = -57.0586;
    public static double START_Y = -43.8964;
    public static double START_HEADING = -126.5;
    public static double PRELOAD_SHOOT_X = -45.3370432609;
    public static double PRELOAD_SHOOT_Y = -24.9996985274;
    public static double PRELOAD_SHOOT_HEADING = -128.71;
    public static double SPIKE_SHOOT_HEADING = -90;
    public static double SPIKE_START_Y = -24.1017;
    public static double SPIKE_RAMP_END_Y = -45.1282;
    public static double SPIKE_TUNNEL_END_Y = -51.1282;
    public static double SPIKE_HEADING = -90.0;
    public static double SPIKE_1_X = -12.3457;
    public static double GATE_X = -7.0;
    public static double GATE_Y = -54.0;
    public static double SPIKE_1_SHOOT_X = -20.3370432609;
    public static double SPIKE_1_SHOOT_Y = -33.9996985274;
    public static double SPIKE_2_SHOOT_X = -12.3370432609;
    public static double SPIKE_2_SHOOT_Y = -16.9996985274;
    public static double SPIKE_3_SHOOT_X = -4.6629567391;
    public static double SPIKE_3_SHOOT_Y = -8.9996985274;
    public static double SPIKE_2_X = 12.3457;
    public static double SPIKE_2_END_X = 13.8457;
    public static double SPIKE_3_X = 34.3457;

    static TrajectoryActionBuilder trajToShoot(TrajectoryActionBuilder builder, int spike) {
        if (spike == 0) {
            Pose2d endRobotPose = new Pose2d(PRELOAD_SHOOT_X, PRELOAD_SHOOT_Y, Math.toRadians(PRELOAD_SHOOT_HEADING));
            return builder.strafeToConstantHeading(endRobotPose.position);
        } else {
            double shootX = spike == 1 ? SPIKE_1_SHOOT_X : (spike == 2 ? SPIKE_2_SHOOT_X : SPIKE_3_SHOOT_X);
            double shootY = spike == 1 ? SPIKE_1_SHOOT_Y : (spike == 2 ? SPIKE_2_SHOOT_Y : SPIKE_3_SHOOT_Y);
            Pose2d endRobotPose = new Pose2d(shootX, shootY, Math.toRadians(SPIKE_SHOOT_HEADING));
            return builder.splineToSplineHeading(
                    endRobotPose,
                    Math.toRadians(-190)
            );
        }
    }

    static TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder, int spike) {
        double currSpikeX = spike == 1 ? SPIKE_1_X : (spike == 2 ? SPIKE_2_X : SPIKE_3_X);
        double endSpikeX = spike == 2 ? SPIKE_2_END_X : currSpikeX;
        double endSpikeY = spike <= 1 ? SPIKE_RAMP_END_Y : SPIKE_TUNNEL_END_Y;

        TrajectoryActionBuilder intake = builder.splineToSplineHeading(
                new Pose2d(endSpikeX, endSpikeY, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(-SPIKE_HEADING),
                new TranslationalVelConstraint(15.0)
        );
        if(spike == 1){
            return intake.endTrajectory().splineToConstantHeading(
                    new Vector2d(GATE_X, GATE_Y),
                    Math.toRadians(SPIKE_HEADING)
            ).strafeTo(new Vector2d(GATE_X, GATE_Y)).waitSeconds(0.5).splineToSplineHeading(
                    new Pose2d(GATE_X, GATE_Y + 5, Math.toRadians((SPIKE_HEADING * 4 + SPIKE_SHOOT_HEADING) / 5.0)),
                    Math.toRadians(-SPIKE_HEADING),
                    new TranslationalVelConstraint(15.0)
            );
        }
        else {
            return intake.splineToSplineHeading(
                    new Pose2d(endSpikeX, endSpikeY + 5, Math.toRadians((SPIKE_HEADING * 4 + SPIKE_SHOOT_HEADING) / 5.0)),
                    Math.toRadians(-SPIKE_HEADING),
                    new TranslationalVelConstraint(15.0)
            );
        }
    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));

        TrajectoryActionBuilder preloadShoot = trajToShoot(myBot.getDrive().actionBuilder(startPose), 0);
//        Action doPreloadShoot = new Robot.ShootSequenceAction();

        VelConstraint toSpike1VelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > SPIKE_1_X - 5.0) {
                return 25.0;
            } else {
                return 50.0;
            }
        };
        TrajectoryActionBuilder toSpike1 = preloadShoot.fresh()
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(
                        new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING),
                        toSpike1VelConstraint
                );

        TrajectoryActionBuilder toSpike1IntakeAndShoot = trajToShoot(intakeFromSpike(toSpike1, 1), 1);
//        Action doSpike1Shoot = new Robot.ShootSequenceAction();

        VelConstraint toSpike2VelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > SPIKE_2_X - 5.0) {
                return 25.0;
            } else {
                return 50.0;
            }
        };
        TrajectoryActionBuilder toSpike2 = toSpike1IntakeAndShoot.fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(SPIKE_2_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING),
                        toSpike2VelConstraint
                );

        TrajectoryActionBuilder toSpike2IntakeAndShoot = trajToShoot(intakeFromSpike(toSpike2, 2), 2);
//        Action doSpike2Shoot = new Robot.ShootSequenceAction();

        VelConstraint toSpike3VelConstraint = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() > SPIKE_3_X - 8.0) {
                return 25.0;
            } else {
                return 50.0;
            }
        };
        TrajectoryActionBuilder toSpike3 = toSpike2IntakeAndShoot.fresh()
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(SPIKE_3_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING),
                        toSpike3VelConstraint
                );

        TrajectoryActionBuilder toSpike3IntakeAndShoot = trajToShoot(intakeFromSpike(toSpike3, 3), 3);
//        Action doSpike3Shoot = new Robot.ShootSequenceAction();

        TrajectoryActionBuilder leaveLaunchZone = toSpike3IntakeAndShoot.fresh().strafeToLinearHeading(
                new Vector2d(SPIKE_3_SHOOT_X, SPIKE_3_SHOOT_Y-20),
                0
        );

        myBot.runAction(new SequentialAction(
                preloadShoot.build(),
                new SleepAction(1.75),
                toSpike1IntakeAndShoot.build(),
                new SleepAction(1.75),
                toSpike2IntakeAndShoot.build(),
                new SleepAction(1.75),
                toSpike3IntakeAndShoot.build(),
                new SleepAction(1.75),
                leaveLaunchZone.build()
        ));

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_DECODE_JUICE_DARK) // You can change this to match your season
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}