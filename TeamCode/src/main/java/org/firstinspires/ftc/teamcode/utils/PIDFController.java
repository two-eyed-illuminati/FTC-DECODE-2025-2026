package org.firstinspires.ftc.teamcode.utils;

public class PIDFController {
    double pCoefficient;
    double dCoefficient;
    double feedforwardCoefficient;
    double oldError = 0;
    public PIDFController(double pCoefficient, double dCoefficient, double feedforwardCoefficient){
        this.pCoefficient = pCoefficient;
        this.dCoefficient = dCoefficient;
        this.feedforwardCoefficient = feedforwardCoefficient;
    }
    public double getPower(double currPos, double targetPos){
        double power = feedforwardCoefficient * targetPos + pCoefficient * (targetPos-currPos) + dCoefficient * (targetPos-currPos - oldError);
        oldError = targetPos-currPos;
        return power;
    }
}
