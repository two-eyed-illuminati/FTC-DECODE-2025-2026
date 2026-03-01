package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;

public class AutoBuilder {
    public static double START_X = -57.0586;
    public static double START_Y = -43.8964;
    public static double START_HEADING = -126.5;
    ArrayList<String> actions;
    ArrayList<Action> actionObjs;
    TrajectoryActionBuilder currentTab;

    public AutoBuilder(){
        this.actions = new ArrayList<>();
        this.actionObjs = new ArrayList<>();
        currentTab = Robot.drive.actionBuilder(new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING)));
    }

    public static double SPIKE_SHOOT_HEADING = -90;
    public static double SPIKE_SHOOT_TANGENT_ANGLE = -190;
    public static double SPIKE_SHOOT_X = -23.3370432609;
    public static double SPIKE_SHOOT_Y = -31.9996985274;
    public static double PRELOAD_SHOOT_X = -24.3370432609;
    public static double PRELOAD_SHOOT_Y = -23.9996985274;
    public AutoBuilder goToShoot(){
        if(actions.get(actions.size()-1).contains("intakeSpike")){
            currentTab = currentTab.splineToSplineHeading(
                    new Pose2d(SPIKE_SHOOT_X, SPIKE_SHOOT_Y, Math.toRadians(SPIKE_SHOOT_HEADING)),
                    Math.toRadians(SPIKE_SHOOT_TANGENT_ANGLE)
            );
        }
        else{
            currentTab = currentTab.strafeToConstantHeading(
                    new Vector2d(PRELOAD_SHOOT_X, PRELOAD_SHOOT_Y)
            );
        }
        actionObjs.add(currentTab.build());
        currentTab = currentTab.fresh();
        actions.add("goToShoot");
        return this;
    }
    public AutoBuilder shoot(){
        actionObjs.add(new Robot.ShootSequenceAction());
        actions.add("shoot");
        return this;
    }

    public static double TO_SPIKE_INITIAL_TANGENT_ANGLE = -135;
    public static double SPIKE_HEADING = -90.0;
    public static double SPIKE_START_Y = -32.1017;
    public static double SPIKE_1_X = -12.3457;
    public AutoBuilder goToSpike1(){
        currentTab = currentTab.setTangent(Math.toRadians(TO_SPIKE_INITIAL_TANGENT_ANGLE)).splineToSplineHeading(
                new Pose2d(SPIKE_1_X, SPIKE_START_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("goToSpike1");
        return this;
    }

    public static double SPIKE_2_X = 12.3457;
    public AutoBuilder goToSpike2(){
        currentTab = currentTab.setTangent(Math.toRadians(TO_SPIKE_INITIAL_TANGENT_ANGLE)).splineToSplineHeading(
                new Pose2d(SPIKE_2_X, SPIKE_START_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("goToSpike2");
        return this;
    }

    public static double SPIKE_3_X = 34.3457;
    public AutoBuilder goToSpike3(){
        currentTab = currentTab.setTangent(Math.toRadians(TO_SPIKE_INITIAL_TANGENT_ANGLE)).splineToSplineHeading(
                new Pose2d(SPIKE_3_X, SPIKE_START_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("goToSpike3");
        return this;
    }


    public static double SPIKE_RAMP_END_Y = -45.1282;
    public static double SPIKE_TUNNEL_END_Y = -58.1282;
    public AutoBuilder intakeSpike1(){
        currentTab = currentTab.splineToSplineHeading(
                new Pose2d(SPIKE_1_X, SPIKE_RAMP_END_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("intakeSpike1");
        return this;
    }
    public static double SPIKE_2_END_X = 13.8457;
    public AutoBuilder intakeSpike2(){
        currentTab = currentTab.splineToSplineHeading(
                new Pose2d(SPIKE_2_END_X, SPIKE_TUNNEL_END_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("intakeSpike2");
        return this;
    }
    public AutoBuilder intakeSpike3(){
        currentTab = currentTab.splineToSplineHeading(
                new Pose2d(SPIKE_3_X, SPIKE_TUNNEL_END_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("intakeSpike3");
        return this;
    }

    public static double GATE_X = -7.0;
    public static double GATE_Y = -55.0;
    public AutoBuilder goToGateHit(){
        currentTab = currentTab.splineToSplineHeading(
                new Pose2d(GATE_X, GATE_Y, SPIKE_HEADING),
                Math.toRadians(SPIKE_HEADING)
        );
        actions.add("goToGateHit");
        return this;
    }
    public Action build(){
        actionObjs.add(currentTab.build());
        return new SequentialAction(actionObjs);
    }
}
