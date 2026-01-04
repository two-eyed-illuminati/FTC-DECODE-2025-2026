package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Autonomous(name="Auto Goal Zone", group="Main")
@Config
public class AutoGoalZone extends LinearOpMode {
    public static double START_X = -58.0586;
    public static double START_Y = -40.7964;
    public static double START_HEADING = -128.71;
    public static double SHOOT_X = -41.0621;
    public static double SHOOT_Y = -29.6497;
    public static double SHOOT_HEADING = -131.9655;
    public static double SPIKE_START_Y = -22.1017;
    public static double SPIKE_END_Y = -51.1282;
    public static double SPIKE_HEADING = -90.0;
    public static double SPIKE_1_X = -14.3457;
    public static double SPIKE_2_X = 10.3457;
    public static double SPIKE_3_X = 34.3457;

    TrajectoryActionBuilder trajToShoot(TrajectoryActionBuilder builder) {
        Pose2d endRobotPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING));
        return builder.afterDisp(0, () -> {
            Robot.aimOuttakeTurret(endRobotPose);
            Robot.shootOuttake(endRobotPose);
        }).strafeToLinearHeading(
                endRobotPose.position,
                Math.toRadians(SHOOT_HEADING)
        );
    }

    TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder){
        return builder.afterDisp(0, () -> {
            Robot.intake.setPower(1.0);
        }).lineToY(SPIKE_END_Y).afterDisp(0, () -> {
            Robot.intake.setPower(0.0);
        }).lineToY(SPIKE_START_Y);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.drive.localizer.setPose(startPose);
        waitForStart();

//        TrajectoryActionBuilder goToOrigin = Robot.drive.actionBuilder(startPose).strafeToLinearHeading(
//                new Vector2d(0, 0),
//                0
//        );
//        Actions.runBlocking(
//                goToOrigin.build()
//        );

        TrajectoryActionBuilder preloadShoot = trajToShoot(Robot.drive.actionBuilder(startPose))
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike1 = preloadShoot.fresh().splineToLinearHeading(
                new Pose2d(SHOOT_X, SHOOT_Y + 10.0, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING/2.0)
        ).splineToLinearHeading(
                new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake1 = intakeFromSpike(toSpike1.fresh());
        TrajectoryActionBuilder spike1Shoot = trajToShoot(intake1.fresh())
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike2 = spike1Shoot.fresh().splineToLinearHeading(
                new Pose2d(SPIKE_2_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake2 = intakeFromSpike(toSpike2.fresh());
        TrajectoryActionBuilder spike2Shoot = trajToShoot(intake2.fresh())
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike3 = spike2Shoot.fresh().splineToLinearHeading(
                new Pose2d(SPIKE_3_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake3 = intakeFromSpike(toSpike3.fresh());
        TrajectoryActionBuilder spike3Shoot = trajToShoot(intake3.fresh())
                .afterDisp(0, new InstantAction(Robot::shootSequence));


        Actions.runBlocking(
                new SequentialAction(
                        preloadShoot.build(),
                        toSpike1.build(),
                        intake1.build(),
                        spike1Shoot.build(),
                        toSpike2.build(),
                        intake2.build(),
                        spike2Shoot.build(),
                        toSpike3.build(),
                        intake3.build(),
                        spike3Shoot.build()
                )
        );
    }
}
