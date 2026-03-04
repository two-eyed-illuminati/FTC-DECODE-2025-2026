package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp(name="Test Distance Sensors", group="Tests")
public class TestDistanceSensors extends OpMode {
    @Override
    public void init() {
        Robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Robot.telemetry.addData("Front", Robot.voltageToDistance(Robot.frontDistanceSensor.getVoltage()));
        Robot.telemetry.addData("Top", Robot.voltageToDistance(Robot.topDistanceSensor.getVoltage()));
    }
}
