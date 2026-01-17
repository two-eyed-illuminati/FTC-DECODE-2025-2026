package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCurrent {
    // --- Copy of your constants ---
    public static double START_X = -58.0586;
    public static double START_Y = -40.7964;
    public static double START_HEADING = -128.71;
    public static double SHOOT_X = -41.0621;
    public static double SHOOT_Y = -29.6497;
    public static double SHOOT_HEADING = -131.9655;
    public static double SPIKE_START_Y = -22.1017;
    public static double SPIKE_RAMP_END_Y = -49.1282;
    public static double SPIKE_TUNNEL_END_Y = -53.1282;
    public static double SPIKE_HEADING = -90.0;
    public static double SPIKE_1_X = -14.3457;
    public static double SPIKE_2_X = 10.3457;
    public static double SPIKE_2_END_X = 11.3457;
    public static double SPIKE_3_X = 34.3457;

    // --- Adapted helper methods (Removed Robot hardware calls) ---
    static TrajectoryActionBuilder trajToShoot(TrajectoryActionBuilder builder, boolean preload) {
        Pose2d endRobotPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING));
        return builder.splineToSplineHeading(
                endRobotPose,
                Math.toRadians(preload ? 180 - SHOOT_HEADING : SHOOT_HEADING)
        );
    }

    static TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder, int spike) {
        double currSpikeX = spike == 1 ? SPIKE_1_X : (spike == 2 ? SPIKE_2_X : SPIKE_3_X);
        double endSpikeX = spike == 2 ? SPIKE_2_END_X : currSpikeX;
        return builder.strafeTo(
                        new Vector2d(endSpikeX, spike <= 1 ? SPIKE_RAMP_END_Y : SPIKE_TUNNEL_END_Y),
                        new TranslationalVelConstraint(25.0)
                )
                .splineToSplineHeading(
                        new Pose2d(endSpikeX, SPIKE_START_Y, Math.toRadians((SPIKE_HEADING + SHOOT_HEADING) / 2.0)),
                        Math.toRadians(-SPIKE_HEADING)
                );
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Robot constraints
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));

        // --- Recreating your Action Logic ---

        // 1. Preload Shoot Path
        TrajectoryActionBuilder preloadShootTab = trajToShoot(myBot.getDrive().actionBuilder(startPose), true);

        // 2. To Spike 1
        TrajectoryActionBuilder toSpike1Tab = preloadShootTab.fresh()
                .splineToSplineHeading(
                        new Pose2d(SHOOT_X, SHOOT_Y + 10.0, Math.toRadians((SPIKE_HEADING + SHOOT_HEADING) / 2.0)),
                        Math.toRadians(0)
                )
                .splineToSplineHeading(
                        new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING)
                );

        TrajectoryActionBuilder toSpike1IntakeAndShootTab = trajToShoot(intakeFromSpike(toSpike1Tab, 1), false);

        // 3. To Spike 2
        TrajectoryActionBuilder toSpike2Tab = toSpike1IntakeAndShootTab.fresh()
                .splineToSplineHeading(
                        new Pose2d(SPIKE_2_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING)
                );

        TrajectoryActionBuilder toSpike2IntakeAndShootTab = trajToShoot(intakeFromSpike(toSpike2Tab, 2), false);

        // 4. Leave
        TrajectoryActionBuilder leaveLaunchZoneTab = toSpike2IntakeAndShootTab.fresh().strafeTo(
                new Vector2d(SHOOT_X + 25, SHOOT_Y)
        );

        // Run it all in sequence
        myBot.runAction(new SequentialAction(
                preloadShootTab.build(),
                new SleepAction(3.5), // Placeholder for doPreloadShoot
                toSpike1IntakeAndShootTab.build(),
                new SleepAction(3.5), // Placeholder for doSpike1Shoot
                toSpike2IntakeAndShootTab.build(),
                new SleepAction(3.5), // Placeholder for doSpike2Shoot
                leaveLaunchZoneTab.build()
        ));

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_DECODE_JUICE_DARK) // You can change this to match your season
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}