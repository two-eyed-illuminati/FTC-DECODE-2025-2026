package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

    public static double SPIKE_CLOSE_SHOOT_HEADING = Math.toRadians(-90);
    public static double SPIKE_CLOSE_SHOOT_TANGENT_ANGLE = Math.toRadians(-225);
    public static double SPIKE_1_CLOSE_SHOOT_TANGENT_ANGLE = Math.toRadians(90);
    public static double SPIKE_CLOSE_SHOOT_X = -12.3370432609;
    public static double SPIKE_CLOSE_SHOOT_Y = -15.9996985274;
    public static double PRELOAD_CLOSE_SHOOT_HEADING = Math.toRadians(-126.5);
    public static double PRELOAD_CLOSE_SHOOT_X = -27.3370432609;
    public static double PRELOAD_CLOSE_SHOOT_Y = -22.9996985274;
    public static double PRELOAD_CLOSE_SHOOT_X_GO_TO_SPIKE_1 = -13.3370432609;
    public static double PRELOAD_CLOSE_SHOOT_Y_GO_TO_SPIKE_1 = -17.9996985274;
    public static double LAST_CLOSE_SHOOT_X = -39.3370432609;
    public static double LAST_CLOSE_SHOOT_Y = -15.9996985274;
    public AutoBuilder goToCloseShoot(String type, String tangentType, String posType){
        Pose2d endPose = (
                actions.isEmpty() ?
                        (posType.equals("1") ? new Pose2d(PRELOAD_CLOSE_SHOOT_X_GO_TO_SPIKE_1, PRELOAD_CLOSE_SHOOT_Y_GO_TO_SPIKE_1, PRELOAD_CLOSE_SHOOT_HEADING) : new Pose2d(PRELOAD_CLOSE_SHOOT_X, PRELOAD_CLOSE_SHOOT_Y, PRELOAD_CLOSE_SHOOT_HEADING)) :
                        (posType.equals("last") ? new Pose2d(LAST_CLOSE_SHOOT_X, LAST_CLOSE_SHOOT_Y, SPIKE_CLOSE_SHOOT_HEADING) : new Pose2d(SPIKE_CLOSE_SHOOT_X, SPIKE_CLOSE_SHOOT_Y, SPIKE_CLOSE_SHOOT_HEADING))
        );
        currentTab = currentTab.afterTime(0,
                new ParallelAction(
                        new InstantAction(() -> {
                            Robot.stopIntake();
                        }),
                        Robot.getAimOuttakeTurretAction(pose2dMapped(endPose)),
                        Robot.getShootOuttakeAction(pose2dMapped(endPose))
                )
        );
        if(posType.equals("avoid1")){
            currentTab = currentTab.splineToSplineHeading(
                    new Pose2d(SPIKE_2_X, BACKUP_Y, Math.toRadians(-90)),
                    SPIKE_BACKUP_TANGENT_ANGLE
            );
        }
        if(type.equals("spline")){
            currentTab = currentTab.splineToSplineHeading(
                    endPose,
                    tangentType.equals("1") ? SPIKE_1_CLOSE_SHOOT_TANGENT_ANGLE : SPIKE_CLOSE_SHOOT_TANGENT_ANGLE
            );
        }
        else if(type.equals("strafe")){
            currentTab = currentTab.strafeToLinearHeading(
                    endPose.position,
                    endPose.heading
            );
        }
        currentTab = currentTab.afterTime(0, new InstantAction(() -> {
            Robot.STOP_SHOOT_OUTTAKE_ACTION = true;
            Robot.STOP_AIM_TURRET_ACTION = true;}));
        actionObjs.add(currentTab.build());
        currentTab = currentTab.fresh();
        actions.add("GoToShoot");
        return this;
    }
    public static double INSIDE_ZONE_X = -50;
    public static double INSIDE_ZONE_Y = -16;
    public AutoBuilder goInsideZone(){
        currentTab = currentTab.strafeTo(new Vector2d(INSIDE_ZONE_X, INSIDE_ZONE_Y));
        actions.add("GoInsideZone");
        return this;
    }
    public static double FAR_SHOOT_TANGENT_ANGLE = Math.toRadians(90);
    public static double FAR_SHOOT_HEADING = Math.toRadians(-90);
    public static double FAR_SHOOT_X = 50.3370432609;
    public static double FAR_SHOOT_Y = -12.9996985274;
    public AutoBuilder goToFarShoot(String type){
        Pose2d endPose = (
                new Pose2d(FAR_SHOOT_X, FAR_SHOOT_Y, FAR_SHOOT_HEADING)
        );
        Pose2d endPoseWithCorrection = (
                new Pose2d(FAR_SHOOT_X, FAR_SHOOT_Y-4, FAR_SHOOT_HEADING)
        ); //For some reason the turret is eternally pointed too much to the left. So this should help.
        currentTab = currentTab.afterTime(0,
                new ParallelAction(
                        new InstantAction(() -> {
                            Robot.stopIntake();
                        }),
                        Robot.getAimOuttakeTurretAction(pose2dMapped(endPoseWithCorrection)),
                        Robot.getShootOuttakeAction(pose2dMapped(endPose))
                )
        );
        if(type.equals("spline")){
            currentTab = currentTab.splineToSplineHeading(
                    endPose,
                    FAR_SHOOT_TANGENT_ANGLE
            );
        }
        else if(type.equals("strafe")){
            currentTab = currentTab.strafeToLinearHeading(
                    endPose.position,
                    endPose.heading
            );
        }
        currentTab = currentTab.afterTime(0, new InstantAction(() -> {
            Robot.STOP_SHOOT_OUTTAKE_ACTION = true;
            Robot.STOP_AIM_TURRET_ACTION = true;}));
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
    public AutoBuilder shoot(double time){
        actionObjs.add(Robot.getShootSequenceAction(time));
        actions.add("Shoot");
        return this;
    }

    public static double TO_SPIKE_INITIAL_TANGENT_ANGLE = Math.toRadians(0.0);
    public static double TO_SPIKE_1_INITIAL_TANGENT_ANGLE = Math.toRadians(-90.0);
    public static double TO_SPIKE_1_FROM_PRELOAD_INITIAL_TANGENT_ANGLE = Math.toRadians(-90.0);
    public static double SPIKE_HEADING = Math.toRadians(-90.0);
    public static double SPIKE_START_Y = -27.1017;
    public static double SPIKE_1_X = -12.3457;
    public static double INTAKE_SPEED = 45.0;
    public AutoBuilder goToSpike1(String type){
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-SPIKE_1_X) < 5.0 && Math.abs(Math.abs(robotPose.position.y.value())-Math.abs(SPIKE_START_Y)) < 7.0){
                return INTAKE_SPEED;
            }
            return 60;
        };
        currentTab = currentTab.afterTime(0, Robot.getReverseIntakeAction());
        currentTab = currentTab.setTangent(type.equals("preload") ? TO_SPIKE_1_FROM_PRELOAD_INITIAL_TANGENT_ANGLE : TO_SPIKE_1_INITIAL_TANGENT_ANGLE).splineToSplineHeading(
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
            if(Math.abs(robotPose.position.x.value()-SPIKE_2_X) < 5.0 && Math.abs(Math.abs(robotPose.position.y.value())-Math.abs(SPIKE_START_Y)) < 7.0){
                return INTAKE_SPEED;
            }
            return 60;
        };
        currentTab = currentTab.setTangent(TO_SPIKE_INITIAL_TANGENT_ANGLE).splineToSplineHeading(
                new Pose2d(SPIKE_2_X, SPIKE_START_Y, SPIKE_HEADING),
                SPIKE_HEADING,
                constraint
        );
        actions.add("GoToSpike2");
        return this;
    }

    public static double SPIKE_3_X = 33.3457;
    public static double TO_SPIKE_3_INITIAL_TANGENT_ANGLE_FAR = Math.toRadians(-150);
    public AutoBuilder goToSpike3(String tangentType){
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-SPIKE_3_X) < 25.0 && Math.abs(Math.abs(robotPose.position.y.value())-Math.abs(SPIKE_START_Y)) < 7.0){
                return INTAKE_SPEED;
            }
            return 60;
        };
        currentTab = currentTab.setTangent(tangentType.equals("far") ? TO_SPIKE_3_INITIAL_TANGENT_ANGLE_FAR : TO_SPIKE_INITIAL_TANGENT_ANGLE).splineToLinearHeading(
                new Pose2d(SPIKE_3_X, SPIKE_START_Y, SPIKE_HEADING),
                SPIKE_HEADING,
                constraint
        );
        actions.add("GoToSpike3");
        return this;
    }
    public AutoBuilder goToSpike3(){
        return goToSpike3("close");
    }


    public static double SPIKE_RAMP_END_Y = -56.1282;
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
    public static double SPIKE_BACKUP_Y = -47.1282;
    public static double SPIKE_BACKUP_TANGENT_ANGLE = Math.toRadians(90.0);
    public AutoBuilder backUpAfterSpike1(){
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_1_X, SPIKE_BACKUP_Y),
                SPIKE_BACKUP_TANGENT_ANGLE
        );
        actions.add("BackUpAfterIntakeSpike1");
        return this;
    }
    public static double SPIKE_2_END_X = 15.3457;
    public static double SPIKE_2_INTAKE_SPEED = 35.0;
    public AutoBuilder intakeSpike2(){
        currentTab = currentTab.afterTime(0, () -> {
            Robot.beginIntake();
        });
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_2_END_X, SPIKE_TUNNEL_END_Y),
                SPIKE_HEADING,
                new TranslationalVelConstraint(SPIKE_2_INTAKE_SPEED)
        );
        actions.add("IntakeSpike2");
        return this;
    }
    public AutoBuilder backUpAfterSpike2(){
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_2_END_X, SPIKE_BACKUP_Y),
                SPIKE_BACKUP_TANGENT_ANGLE
        );
        actions.add("BackUpAfterIntakeSpike2");
        return this;
    }
    public static double BACKUP_Y = -40.0;
    public AutoBuilder backUpToAvoidSpike1(){
        if(!actions.isEmpty() && actions.get(actions.size()-1).equals("GoToGateHit")){
            currentTab = currentTab.setTangent(Math.toRadians(90));
        }
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(SPIKE_2_X, BACKUP_Y),
                SPIKE_BACKUP_TANGENT_ANGLE
        );
        actions.add("BackUp");
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

    public static double GATE_X_LEFT = -4.5;
    public static double GATE_X_RIGHT = 10;
    public static double GATE_Y_BEFORE_HIT = -43.0;
    public static double GATE_Y_HIT = -56.0;
    public static double GATE_HIT_TIME = 0.1;
    public static double GATE_HIT_SPEED = 25.0;
    public AutoBuilder goToGateHit(String side){
        if(!actions.isEmpty() && (actions.get(actions.size()-1).equals("GoToShoot") || actions.get(actions.size()-1).equals("Shoot"))){
            currentTab = currentTab.setTangent(Math.toRadians(0));
        }
        double GATE_X = side.equals("left") ? GATE_X_LEFT : GATE_X_RIGHT;
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(robotPose.position.x.value()-GATE_X) < 5.0 && Math.abs(Math.abs(robotPose.position.y.value())-Math.abs(GATE_Y_BEFORE_HIT)) < 10.0){
                return GATE_HIT_SPEED;
            }
            return 60;
        };
        currentTab = currentTab.splineToConstantHeading(
                new Vector2d(GATE_X, GATE_Y_BEFORE_HIT),
                SPIKE_HEADING,
                constraint
        ).splineToConstantHeading(
                new Vector2d(GATE_X, GATE_Y_HIT),
                SPIKE_HEADING,
                new TranslationalVelConstraint(GATE_HIT_SPEED)
        );
        if(GATE_HIT_TIME > 0){
            currentTab = currentTab.waitSeconds(GATE_HIT_TIME);
        }
        actions.add("GoToGateHit");
        return this;
    }
    public static double GATE_INTAKE_X = 12.25;
    public static double GATE_INTAKE_Y = -57.5;
    public static double GATE_INTAKE_HEADING = Math.toRadians(-114);
    public static double GATE_INTAKE_TIME = 1.2;
    public AutoBuilder intakeFromGate(){
        currentTab = currentTab.afterTime(0, () -> {
            Robot.beginIntake();
        });
        if(!actions.isEmpty() && actions.get(actions.size()-1).equals("GoToGateHit")){
            currentTab = currentTab.setTangent(Math.toRadians(20));
        }
        if(!actions.isEmpty() && actions.get(actions.size()-1).equals("Shoot")){
            currentTab = currentTab.setTangent(Math.toRadians(-20));
        }
        VelConstraint constraint = (robotPose, _path, _disp) -> {
            if(Math.abs(Math.abs(robotPose.position.y.value())-Math.abs(GATE_INTAKE_Y)) < 10.0){
                return GATE_HIT_SPEED;
            }
            return 60;
        };
        currentTab = currentTab.splineToLinearHeading(
                new Pose2d(GATE_INTAKE_X, GATE_INTAKE_Y, GATE_INTAKE_HEADING),
                Math.toRadians(-90),
                constraint
        );
        currentTab = currentTab.waitSeconds(GATE_INTAKE_TIME);
//        currentTab = currentTab.stopAndAdd(Robot.getCorrectSecondsAction(pose2dMapped(new Pose2d(GATE_INTAKE_X, GATE_INTAKE_Y, GATE_INTAKE_HEADING)), GATE_INTAKE_TIME)).setTangent(Math.toRadians(90));
        actions.add("IntakeFromGate");
        return this;
    }
    public static double LOOSE_INTAKE_START_Y = -43.0;
    public static double LOOSE_INTAKE_START_HEADING = Math.toRadians(-100.0);
    public AutoBuilder looseIntake(double x){
        currentTab = currentTab.strafeToLinearHeading(
                new Vector2d(x, LOOSE_INTAKE_START_Y), LOOSE_INTAKE_START_HEADING
        );
        actionObjs.add(currentTab.build());
        actionObjs.add(Robot.getLooseIntakeAction(
                pose2dMapped(
                        new Pose2d(FAR_SHOOT_X, FAR_SHOOT_Y, FAR_SHOOT_HEADING)
                )
        ));
        currentTab = Robot.drive.actionBuilder(new Pose2d(FAR_SHOOT_X, FAR_SHOOT_Y, FAR_SHOOT_HEADING), currentTab.getPoseMap());
        actions.add("LooseIntake");
        return this;
    }
    public static double CORNER_START_X = 56.0;
    public static double CORNER_END_X = 64.0;
    public static double CORNER_Y = -59.25;
    public static double CORNER_START_HEADING = Math.toRadians(-65);
    public static double CORNER_END_HEADING = Math.toRadians(-90);
    public AutoBuilder intakeFromCorner(){
        currentTab = currentTab.afterTime(0, new InstantAction(() -> {Robot.beginIntake();}));
        currentTab = currentTab.waitSeconds(0.5);
        currentTab = currentTab.strafeToLinearHeading(new Vector2d(CORNER_START_X, CORNER_Y), CORNER_START_HEADING);
        currentTab = currentTab.strafeTo(new Vector2d(CORNER_END_X, CORNER_Y));
        currentTab = currentTab.strafeToLinearHeading(new Vector2d(CORNER_END_X, Math.signum(CORNER_Y)*(Math.abs(CORNER_Y)+0.01)), CORNER_END_HEADING);
        currentTab = currentTab.strafeTo(new Vector2d(CORNER_END_X, Math.signum(CORNER_Y)*(Math.abs(CORNER_Y)+1)));
        actions.add("IntakeFromCorner");
        return this;
    }
    public static double OUTSIDE_ZONE_Y = -40.0;
    public AutoBuilder leaveZone(){
        currentTab = currentTab.setTangent(Math.toRadians(-90)).lineToY(OUTSIDE_ZONE_Y);
        actions.add("LeaveZone");
        return this;
    }
    public Action build(){
        actionObjs.add(currentTab.build());
        return new SequentialAction(actionObjs);
    }
}