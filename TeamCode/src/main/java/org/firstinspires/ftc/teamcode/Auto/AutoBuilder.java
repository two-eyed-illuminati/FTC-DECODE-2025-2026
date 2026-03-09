package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;

public class AutoBuilder {
    ArrayList<String> actions;
    ArrayList<Action> actionObjs;
    TrajectoryActionBuilder currentTab;
    public AutoBuilder(TrajectoryActionBuilder tab){
        this.actions = new ArrayList<>();
        this.actionObjs = new ArrayList<>();
        currentTab = tab;
    }
    public static Pose2d pose2dMapped(Pose2d pose){
        if(Robot.alliance == Robot.Alliance.BLUE){
            return pose;
        }
        else{
            return new Pose2d(pose.position.x, -pose.position.y, -pose.heading.log());
        }
    }

    public static double SPIKE_SHOOT_HEADING = Math.toRadians(-90);
    public static double SPIKE_SHOOT_TANGENT_ANGLE = Math.toRadians(-225);
    public static double SPIKE_SHOOT_X = -23.3370432609;
    public static double SPIKE_SHOOT_Y = -31.9996985274;
    public static double PRELOAD_SHOOT_HEADING = Math.toRadians(-126.5);
    public static double PRELOAD_SHOOT_X = -24.3370432609;
    public static double PRELOAD_SHOOT_Y = -23.9996985274;
    public AutoBuilder goToShoot(String type){
        Pose2d endPose = (
                actions.isEmpty() ?
                        new Pose2d(PRELOAD_SHOOT_X, PRELOAD_SHOOT_Y, PRELOAD_SHOOT_HEADING) :
                        new Pose2d(SPIKE_SHOOT_X, SPIKE_SHOOT_Y, SPIKE_SHOOT_HEADING)
        );
        currentTab = currentTab.afterTime(0,
                new ParallelAction(
                    Robot.getAimOuttakeTurretAction(pose2dMapped(endPose)),
                    Robot.getShootOuttakeAction(pose2dMapped(endPose))
                )
        );
        if(type.equals("spline")){
            currentTab = currentTab.splineToSplineHeading(
                    endPose,
                    SPIKE_SHOOT_TANGENT_ANGLE
            );
        }
        else if(type.equals("strafe")){
            currentTab = currentTab.strafeToConstantHeading(
                    endPose.position
            );
        }
        actionObjs.add(currentTab.build());
        currentTab = currentTab.fresh();
        actions.add("GoToShoot");
        return this;
    }
    public AutoBuilder shoot(){
        actionObjs.add(Robot.getShootSequenceAction());
        actions.add("Shoot");
        return this;
    }

    public static double TO_SPIKE_INITIAL_TANGENT_ANGLE = Math.toRadians(20.0);
    public static double SPIKE_HEADING = Math.toRadians(-90.0);
    public static double SPIKE_START_Y = -29.1017;
    public static double SPIKE_1_X = -12.3457;
    public static double INTAKE_SPEED = 25.0;
    public AutoBuilder goToSpike1(){
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-SPIKE_1_X) < 5.0){
                return INTAKE_SPEED;
            }
            return 50;
        };
        currentTab = currentTab.setTangent(TO_SPIKE_INITIAL_TANGENT_ANGLE).splineToSplineHeading(
                new Pose2d(SPIKE_1_X, SPIKE_START_Y, SPIKE_HEADING),
                SPIKE_HEADING,
                constraint
        );
        actions.add("GoToSpike1");
        return this;
    }

    public static double SPIKE_2_X = 12.3457;
    public AutoBuilder goToSpike2(){
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-SPIKE_2_X) < 5.0){
                return INTAKE_SPEED;
            }
            return 50;
        };
        currentTab = currentTab.setTangent(TO_SPIKE_INITIAL_TANGENT_ANGLE).splineToSplineHeading(
                new Pose2d(SPIKE_2_X, SPIKE_START_Y, SPIKE_HEADING),
                SPIKE_HEADING,
                constraint
        );
        actions.add("GoToSpike2");
        return this;
    }

    public static double SPIKE_3_X = 34.3457;
    public AutoBuilder goToSpike3(){
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-SPIKE_3_X) < 5.0){
                return INTAKE_SPEED;
            }
            return 50;
        };
        currentTab = currentTab.setTangent(TO_SPIKE_INITIAL_TANGENT_ANGLE).splineToSplineHeading(
                new Pose2d(SPIKE_3_X, SPIKE_START_Y, SPIKE_HEADING),
                SPIKE_HEADING,
                constraint
        );
        actions.add("GoToSpike3");
        return this;
    }


    public static double SPIKE_RAMP_END_Y = -55.1282;
    public static double SPIKE_TUNNEL_END_Y = -62.1282;
    public AutoBuilder intakeSpike1(){
        currentTab = currentTab.afterTime(0, () -> {
            Robot.beginIntake();
        });
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_1_X, SPIKE_RAMP_END_Y),
                SPIKE_HEADING,
                new TranslationalVelConstraint(INTAKE_SPEED)
        );
        actions.add("IntakeSpike1");
        return this;
    }
    public static double SPIKE_2_END_X = 13.8457;
    public AutoBuilder intakeSpike2(){
        currentTab = currentTab.afterTime(0, () -> {
            Robot.beginIntake();
        });
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_2_END_X, SPIKE_TUNNEL_END_Y),
                SPIKE_HEADING,
                new TranslationalVelConstraint(INTAKE_SPEED)
        );
        actions.add("IntakeSpike2");
        return this;
    }
    public static double SPIKE_BACKUP_Y = -52.1282;
    public static double SPIKE_BACKUP_TANGENT_ANGLE = Math.toRadians(90.0);
    public AutoBuilder backUpAfterSpike2(){
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_2_END_X, SPIKE_BACKUP_Y),
                SPIKE_BACKUP_TANGENT_ANGLE
        );
        actions.add("backUpAfterIntakeSpike2");
        return this;
    }
    public AutoBuilder intakeSpike3(){
        currentTab = currentTab.afterTime(0, () -> {
            Robot.beginIntake();
        });
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_3_X, SPIKE_TUNNEL_END_Y),
                SPIKE_HEADING,
                new TranslationalVelConstraint(INTAKE_SPEED)
        );
        actions.add("IntakeSpike3");
        return this;
    }
    public AutoBuilder backUpAfterSpike3(){
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_3_X, SPIKE_BACKUP_Y),
                SPIKE_BACKUP_TANGENT_ANGLE
        );
        actions.add("BackUpAfterIntakeSpike3");
        return this;
    }

    public static double GATE_X_LEFT = -6.5;
    public static double GATE_X_RIGHT = 8.5;
    public static double GATE_Y_BEFORE_HIT = -50.0;
    public static double GATE_Y_HIT = -55.0;
    public static double GATE_HIT_TIME = 0.05;
    public AutoBuilder goToGateHit(String side){
        double GATE_X = side.equals("left") ? GATE_X_LEFT : GATE_X_RIGHT;
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-GATE_X) < 5.0){
                return 15.0;
            }
            return 50;
        };
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(GATE_X, GATE_Y_BEFORE_HIT),
                SPIKE_HEADING,
                constraint
        ).splineToConstantHeading(
                new Vector2d(GATE_X, GATE_Y_HIT),
                SPIKE_HEADING,
                new TranslationalVelConstraint(15.0)
        ).waitSeconds(GATE_HIT_TIME);
        actions.add("GoToGateHit");
        return this;
    }
    public Action build(){
        actionObjs.add(currentTab.build());
        return new SequentialAction(actionObjs);
    }
}