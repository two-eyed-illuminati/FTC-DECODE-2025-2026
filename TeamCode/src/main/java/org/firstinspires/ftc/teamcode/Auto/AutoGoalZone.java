package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Autonomous(name="Auto Goal Zone", group="Main")
public class AutoGoalZone extends LinearOpMode {
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 125.0;
    public static double SHOOT_X = 0.0;
    public static double SHOOT_Y = 0.0;
    public static double SHOOT_HEADING = 125.0;
    public static double SPIKE_START_Y = 0.0;
    public static double SPIKE_END_Y = 0.0;
    public static double SPIKE_HEADING = 90.0;
    public static double SPIKE_1_X = 0.0;

    TrajectoryActionBuilder trajToShoot(TrajectoryActionBuilder builder) {
        Pose2d endRobotPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING));
        return builder.afterDisp(0, () -> {
            Robot.aimOuttakeTurret(endRobotPose);
            Robot.shootOuttake(endRobotPose);
        }).splineToLinearHeading(
                endRobotPose,
                Math.toRadians(SHOOT_HEADING)
        );
    }

    TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder){
        return builder.afterDisp(0, () -> {
            Robot.intake.setPower(1);
            Robot.transfer.motor.setPower(1);
        }).lineToY(SPIKE_END_Y).afterDisp(0, () -> {
            Robot.intake.setPower(0);
            Robot.transfer.motor.setPower(0);
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Pose2d shootPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.drive.localizer.setPose(startPose);
        waitForStart();

        TrajectoryActionBuilder preloadShoot = trajToShoot(Robot.drive.actionBuilder(startPose))
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike1 = preloadShoot.fresh().splineToLinearHeading(
                new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(90)
        );
        TrajectoryActionBuilder intake1 = intakeFromSpike(toSpike1.fresh());
        TrajectoryActionBuilder spike1Shoot = trajToShoot(intake1.fresh())
                .afterDisp(0, new InstantAction(Robot::shootSequence));


        Actions.runBlocking(
                new SequentialAction(
                        preloadShoot.build(),
                        toSpike1.build(),
                        intake1.build(),
                        spike1Shoot.build()
                )
        );
    }
}
