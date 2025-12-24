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
    public static double START_HEADING = 0.0;
    public static double SHOOT_X = 0.0;
    public static double SHOOT_Y = 0.0;
    public static double SHOOT_HEADING = 0.0;

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

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Pose2d shootPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.drive.localizer.setPose(startPose);
        waitForStart();

        TrajectoryActionBuilder trajToShoot1 = trajToShoot(Robot.drive.actionBuilder(startPose));

        Action combinedAction = new SequentialAction(
                trajToShoot1.build(),
                new InstantAction(
                        () -> {
                            Robot.shootSequence();
                        }
                )
        );

        Actions.runBlocking(
                combinedAction
        );
    }
}
