package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    double pCoefficient;
    double dCoefficient;
    double feedforwardCoefficient;
    double deltaTargetPCoefficient;
    double oldCurrPos = 0;
    double oldDeltaCurrPos = 0;
    double oldTargetPos = 0;
    double oldDeltaTargetPos = 0;
    ElapsedTime timeSinceOld;
    public PIDFController(double pCoefficient, double dCoefficient, double feedforwardCoefficient, double deltaTargetPCoefficient){
        this.pCoefficient = pCoefficient;
        this.dCoefficient = dCoefficient;
        this.feedforwardCoefficient = feedforwardCoefficient;
        this.deltaTargetPCoefficient = deltaTargetPCoefficient;
        timeSinceOld = new ElapsedTime();
    }
    public double getPower(double currPos, double targetPos){
        double alpha = 0.8;
        double deltaCurrPos = ((currPos - oldCurrPos) / timeSinceOld.seconds()) * alpha + oldDeltaCurrPos * (1-alpha);
        double deltaTargetPos = ((targetPos - oldTargetPos) / timeSinceOld.seconds()) * alpha + oldDeltaTargetPos * (1-alpha);

        double power = (
                        feedforwardCoefficient * targetPos +
                        pCoefficient * (targetPos-currPos) +
                        dCoefficient * (-deltaCurrPos) +
                        deltaTargetPCoefficient * deltaTargetPos

        );

        oldCurrPos = currPos;
        oldTargetPos = targetPos;
        oldDeltaCurrPos = deltaCurrPos;
        oldDeltaTargetPos = deltaTargetPos;

        timeSinceOld.reset();
        return power;
    }
}
