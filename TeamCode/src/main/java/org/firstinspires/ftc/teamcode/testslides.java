package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialLeviathanCode.programmingBoard.ProgrammingBoardV1;

@TeleOp()
public class testslides extends OpMode{
    SlidesBoard slides = new SlidesBoard();


    @Override
    public void init(){

        slides.init(hardwareMap);

    }

    @Override
    public void loop(){
        slides.setPosition(6800);
    }

}
