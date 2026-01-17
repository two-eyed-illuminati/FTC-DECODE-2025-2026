package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Autonomous(name="Auto Goal Zone", group="Main")
@Config
public class AutoGoalZone extends LinearOpMode {
    public static double START_X = -57.0586;
    public static double START_Y = -43.8964;
    public static double START_HEADING = -126.5;
    public static double PRELOAD_SHOOT_X = -45.3370432609;
    public static double PRELOAD_SHOOT_Y = -24.9996985274;
    public static double PRELOAD_SHOOT_HEADING = -128.71;
    public static double SPIKE_SHOOT_X = -20.3370432609;
    public static double SPIKE_SHOOT_Y = -24.9996985274;
    public static double SPIKE_SHOOT_HEADING = -90;
    public static double SPIKE_START_Y = -22.1017;
    public static double SPIKE_RAMP_END_Y = -52.1282;
    public static double SPIKE_TUNNEL_END_Y = -58.1282;
    public static double SPIKE_HEADING = -90.0;
    public static double SPIKE_1_X = -12.3457;
    public static double SPIKE_2_X = 11.3457;
    public static double SPIKE_2_END_X = 12.8457;
    public static double SPIKE_3_X = 34.3457;

    TrajectoryActionBuilder trajToShoot(TrajectoryActionBuilder builder, boolean preload) {
        if (preload) {
            Pose2d endRobotPose = new Pose2d(PRELOAD_SHOOT_X, PRELOAD_SHOOT_Y, Math.toRadians(PRELOAD_SHOOT_HEADING));
            return builder.afterDisp(0, () -> {
                Robot.aimOuttakeTurret(endRobotPose);
                Robot.shootOuttake(endRobotPose, false);
            }).strafeToConstantHeading(endRobotPose.position);
        } else {
            Pose2d endRobotPose = new Pose2d(SPIKE_SHOOT_X, SPIKE_SHOOT_Y, Math.toRadians(SPIKE_SHOOT_HEADING));
            return builder.afterDisp(0, () -> {
                Robot.aimOuttakeTurret(endRobotPose);
                Robot.shootOuttake(endRobotPose, false);
            }).splineToSplineHeading(
                    endRobotPose,
                    Math.toRadians(-190)
            );
        }
    }

    TrajectoryActionBuilder intakeFromSpike(TrajectoryActionBuilder builder, int spike) {
        double currSpikeX = spike == 1 ? SPIKE_1_X : (spike == 2 ? SPIKE_2_X : SPIKE_3_X);
        double endSpikeX = spike == 2 ? SPIKE_2_END_X : currSpikeX;
        double endSpikeY = spike <= 1 ? SPIKE_RAMP_END_Y : SPIKE_TUNNEL_END_Y;

        return builder.splineToSplineHeading(
                        new Pose2d(endSpikeX, endSpikeY, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(-SPIKE_HEADING),
                        new TranslationalVelConstraint(25.0)
                )
                .splineToSplineHeading(
                        new Pose2d(endSpikeX, endSpikeY + 5, Math.toRadians((SPIKE_HEADING * 4 + SPIKE_SHOOT_HEADING) / 5.0)),
                        Math.toRadians(-SPIKE_HEADING)
                )
                .afterDisp(0, () -> {
                    Robot.intake.setPower(0.0);
                    Robot.transfer.setPos(0, 0);
                });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        Robot.initialize(hardwareMap, telemetry);
        Robot.Alliance selectedAlliance = null;
        while(!isStarted()){
            Robot.telemetry.addLine("Press X for BLUE alliance, B for RED alliance");
            if(selectedAlliance == null) {
                Robot.telemetry.addLine("WARNING: No current alliance selected!");
            }
            else{
                Robot.telemetry.addData("Current Alliance", selectedAlliance == Robot.Alliance.BLUE ? "BLUE" : "RED");
            }
            Robot.telemetry.update();
            if(gamepad1.x){
                selectedAlliance = Robot.Alliance.BLUE;
            }
            else if(gamepad1.b){
                selectedAlliance = Robot.Alliance.RED;
            }
        }
        if(selectedAlliance != null){
            Robot.alliance = selectedAlliance;
        }
        else{
            return;
        }

        PoseMap poseMap = Robot.alliance == Robot.Alliance.BLUE ? new IdentityPoseMap() :
                pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());

        Robot.drive.localizer.setPose(Robot.alliance == Robot.Alliance.BLUE ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble()));

        TrajectoryActionBuilder preloadShoot = trajToShoot(Robot.drive.actionBuilder(startPose, poseMap), true);
        Action doPreloadShoot = new Robot.ShootSequenceAction();

        TrajectoryActionBuilder toSpike1 = preloadShoot.fresh()
                .afterDisp(0, () -> {
                    Robot.intake.setPower(1.0);
                    Robot.transfer.setPos(0, 0.1*Robot.transfer.maxVel);
                    Robot.outtake.setPos(0, -1440.0);
                })
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(
                        new Pose2d(SPIKE_1_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING)
                );

        TrajectoryActionBuilder toSpike1IntakeAndShoot = trajToShoot(intakeFromSpike(toSpike1, 1), false);
        Action doSpike1Shoot = new Robot.ShootSequenceAction();

        TrajectoryActionBuilder toSpike2 = toSpike1IntakeAndShoot.fresh()
                .afterDisp(0, () -> {
                    Robot.intake.setPower(1.0);
                    Robot.transfer.setPos(0, 0.1*Robot.transfer.maxVel);
                    Robot.outtake.setPos(0, -1440.0);
                })
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(SPIKE_2_X, SPIKE_START_Y, Math.toRadians(SPIKE_HEADING)),
                        Math.toRadians(SPIKE_HEADING)
                );

        TrajectoryActionBuilder toSpike2IntakeAndShoot = trajToShoot(intakeFromSpike(toSpike2, 2), false);
        Action doSpike2Shoot = new Robot.ShootSequenceAction();

        TrajectoryActionBuilder leaveLaunchZone = toSpike2IntakeAndShoot.fresh().strafeToLinearHeading(
                new Vector2d(SPIKE_SHOOT_X + 20, SPIKE_SHOOT_Y),
                0
        );

        Actions.runBlocking(
                new SequentialAction(
                        preloadShoot.build(),
                        doPreloadShoot,
                        toSpike1IntakeAndShoot.build(),
                        doSpike1Shoot,
                        toSpike2IntakeAndShoot.build(),
                        doSpike2Shoot,
                        leaveLaunchZone.build()
                )
        );
    }
}
