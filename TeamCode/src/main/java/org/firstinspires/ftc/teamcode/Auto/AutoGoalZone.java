package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
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
    public static double SPIKE_RAMP_END_Y = -51.1282;
    public static double SPIKE_TUNNEL_END_Y = -55.1282;
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

    TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder, int spike){
        return builder.afterDisp(0, () -> {
            Robot.intake.setPower(1.0);
        }).lineToY(spike <= 1 ? SPIKE_RAMP_END_Y : SPIKE_TUNNEL_END_Y).afterDisp(0, () -> {
            Robot.intake.setPower(0.0);
        }).lineToY(SPIKE_START_Y);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.drive.localizer.setPose(startPose);
        while(!isStarted()){
            if(gamepad1.x){
                Robot.alliance = Robot.Alliance.BLUE;
            }
            else if(gamepad1.b){
                Robot.alliance = Robot.Alliance.RED;
            }
        }

        PoseMap poseMap = Robot.alliance == Robot.Alliance.BLUE ? new IdentityPoseMap() :
                pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());

        TrajectoryActionBuilder preloadShoot = trajToShoot(Robot.drive.actionBuilder(startPose, poseMap))
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike1 = preloadShoot.fresh().splineToLinearHeading(
                new Pose2d(SHOOT_X, SHOOT_Y + 10.0, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING/2.0)
        ).splineToLinearHeading(
                new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake1 = intakeFromSpike(toSpike1.fresh(), 1);
        TrajectoryActionBuilder spike1IntakeAndShoot = trajToShoot(intake1)
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike2 = spike1IntakeAndShoot.fresh().splineToLinearHeading(
                new Pose2d(SPIKE_2_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake2 = intakeFromSpike(toSpike2.fresh(), 2);
        TrajectoryActionBuilder spike2IntakeAndShoot = trajToShoot(intake2)
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder toSpike3 = spike2IntakeAndShoot.fresh().splineToLinearHeading(
                new Pose2d(SPIKE_3_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                Math.toRadians(SPIKE_HEADING)
        );
        TrajectoryActionBuilder intake3 = intakeFromSpike(toSpike3.fresh(), 3);
        TrajectoryActionBuilder spike3IntakeAndShoot = trajToShoot(intake3)
                .afterDisp(0, new InstantAction(Robot::shootSequence));
        TrajectoryActionBuilder leaveLaunchZone = spike3IntakeAndShoot.fresh().strafeTo(
                new Vector2d(SHOOT_X+25, SHOOT_Y)
        );


        Actions.runBlocking(
                new SequentialAction(
                        preloadShoot.build(),
                        toSpike1.build(),
                        spike1IntakeAndShoot.build(),
                        toSpike2.build(),
                        spike2IntakeAndShoot.build(),
                        toSpike3.build(),
                        spike3IntakeAndShoot.build(),
                        leaveLaunchZone.build()
                )
        );
    }
}
