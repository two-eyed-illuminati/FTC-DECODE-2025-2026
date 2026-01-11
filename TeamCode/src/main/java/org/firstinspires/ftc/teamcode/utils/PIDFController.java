package org.firstinspires.ftc.teamcode.utils;

public class PIDFController {
    double pCoefficient;
    double feedforwardCoefficient;
    public PIDFController(double pCoefficient, double feedforwardCoefficient){
        this.pCoefficient = pCoefficient;
        this.feedforwardCoefficient = feedforwardCoefficient;
    }
    double getPower(double currPos, double targetPos){
        return feedforwardCoefficient * targetPos + pCoefficient * (targetPos-currPos);
    }
}
