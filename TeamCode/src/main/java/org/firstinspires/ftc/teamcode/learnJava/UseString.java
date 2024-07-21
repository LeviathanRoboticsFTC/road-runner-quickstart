package org.firstinspires.ftc.teamcode.learnJava;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class UseString extends OpMode{
    @Override
    public void init(){
        int myAge = 5;
        String myName = "Evan Huang";

        telemetry.addData("Hello", myName);
    }
    @Override
    public void loop(){

    }
}
