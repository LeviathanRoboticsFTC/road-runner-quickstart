package org.firstinspires.ftc.teamcode.officialLeviathanCode.teleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialLeviathanCode.programmingBoard.ProgrammingBoardV1;

@TeleOp()
public class DrivetrainTestv1 extends OpMode{
    ProgrammingBoardV1 board = new ProgrammingBoardV1();
    double speed;
    double turn;
    double strafe;

    @Override
    public void init(){

        board.init(hardwareMap);
        speed = 0;
        turn = 0;
        strafe = 0;
    }

    @Override
    public void loop(){
        speed = 0.6*gamepad1.left_stick_y;
        turn = -0.6*gamepad1.right_stick_x;
        strafe = -0.6*gamepad1.left_stick_x;
        board.driveMotors(speed, turn, strafe);
        if (speed == 0 && turn == 0 && strafe == 0){
            board.stopMotor();
        }
    }

}
