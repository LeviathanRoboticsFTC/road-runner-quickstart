package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ObjectOrientedTestV1 extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        drive test = new drive(3,5, hardwareMap, telemetry);

        waitForStart();
        test.foward();
    }
}
