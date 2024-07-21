package org.firstinspires.ftc.teamcode.learnJava;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class MainAutonTest extends OpMode {


    @Override
    public void init() {

        Point.lifespan = 10;


    }

    @Override
    public void loop(){
        telemetry.addData("lifespan", Point.lifespan);

    }

}
