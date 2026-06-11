package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    public double pCoefficient;
    public double dCoefficient;
    public double feedforwardCoefficient;
    public double oldError = 0;
    public ElapsedTime timeSinceOld;
    public PIDFController(double pCoefficient, double dCoefficient, double feedforwardCoefficient){
        this.pCoefficient = pCoefficient;
        this.dCoefficient = dCoefficient;
        this.feedforwardCoefficient = feedforwardCoefficient;
        timeSinceOld = new ElapsedTime();
    }
    public double getPower(double currPos, double targetPos){
        double power = feedforwardCoefficient * targetPos / Robot.drive.voltageSensor.getVoltage() + pCoefficient * (targetPos-currPos) + dCoefficient * (targetPos-currPos - oldError) / timeSinceOld.seconds();
        oldError = targetPos-currPos;
        timeSinceOld.reset();
        return power;
    }
}
