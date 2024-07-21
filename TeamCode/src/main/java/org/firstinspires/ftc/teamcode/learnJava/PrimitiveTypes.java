package org.firstinspires.ftc.teamcode.learnJava;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class PrimitiveTypes extends OpMode {
    @Override
    public void init() {
        int teamNumber = 25667;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor Pressed", touchSensorPressed);
    }

    @Override
    public void loop(){

    }
}
