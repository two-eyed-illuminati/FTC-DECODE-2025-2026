package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepCurrent {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Robot constraints
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(15, 18)
                .build();

        double START_X = -57.0586;
        double START_Y = -43.8964;
        double START_HEADING = -126.5;

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));

        AutoBuilder autoBuilder = new AutoBuilder(myBot.getDrive().actionBuilder(startPose));

        autoBuilder
                .goToShoot()
                .shoot();
        autoBuilder
                .goToSpike1()
                .intakeSpike1()
                .goToGateHit()
                .goToShoot()
                .shoot();
        autoBuilder
                .goToSpike2()
                .intakeSpike2()
                .backUpAfterSpike2()
                .goToGateHit()
                .goToShoot()
                .shoot();
        autoBuilder
                .goToSpike3()
                .intakeSpike3()
                .backUpAfterSpike3()
                .goToShoot()
                .shoot();

        myBot.runAction(autoBuilder.build());

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_DECODE_JUICE_DARK) // You can change this to match your season
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}