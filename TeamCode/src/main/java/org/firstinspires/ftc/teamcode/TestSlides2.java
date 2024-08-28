package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.officialLeviathanCode.programmingBoard.ProgrammingBoardV1;

@TeleOp()
public class TestSlides2 extends LinearOpMode {
    SlidesBoard slides = new SlidesBoard();

    @Override
    public void runOpMode() throws InterruptedException {
        slides.init(hardwareMap);

        waitForStart();
        SlidesBoard.SlidesToPosition a = slides.new SlidesToPosition(0, telemetry);

        while(opModeIsActive()){
            if(gamepad1.a && slides.slidesMotor.getTargetPosition()!=0){
                a.CancelSlides();
                a = slides.new SlidesToPosition(0, telemetry);
            }else if(gamepad1.b && slides.slidesMotor.getTargetPosition()!=6800){
                a.CancelSlides();
                a = slides.new SlidesToPosition(6800, telemetry);

            }else if(gamepad1.dpad_down){
                a.CancelSlides();

            }

            TelemetryPacket packet=new TelemetryPacket();
            a.run(packet);
        }

        /*Actions.runBlocking(
                new SequentialAction(
                        slides.new SlidesToPosition(6800, telemetry),
                        slides.new SlidesToPosition(0, telemetry)
                )
        );

         */

        telemetry.addData("Oops", ":(");
        telemetry.update();

    }
}