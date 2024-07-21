package org.firstinspires.ftc.teamcode.learnJava.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp()

public class MotorManualTest extends OpMode{

   ProgrammingBoardTest board = new ProgrammingBoardTest();
    @Override
    public void init(){
        board.init(hardwareMap);
    }
    @Override
    public void loop(){
        if(board.getDistance(DistanceUnit.CM)<10){
            board.stopMotor();
        }else{
            board.setDriveFowardSpeed(0.5);
        }

    }
}
