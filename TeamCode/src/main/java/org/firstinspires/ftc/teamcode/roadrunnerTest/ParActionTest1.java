package org.firstinspires.ftc.teamcode.roadrunnerTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SlidesBoard;

@TeleOp
public class ParActionTest1 extends LinearOpMode {

    //class for waiting until a is pressed
    public final class WaitForButton implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(gamepad1.a==true){
                return false;
            }

            return true;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        SlidesBoard slides = new SlidesBoard();
        slides.init(hardwareMap);

        Action action1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(23,23))
                .build();
        Action action4 = drive.actionBuilder(new Pose2d(23, 23, 0))
                .strafeTo(new Vector2d(0,0))
                .build();
        Action action5 = new WaitForButton();
        SlidesBoard.SlidesToPosition action2 = slides.new SlidesToPosition(6800, telemetry);
        SlidesBoard.SlidesToPosition action3 = slides.new SlidesToPosition(0, telemetry);

        waitForStart();

        //go up and wait for button at the same time, when a is pressed, finish going up, then lower down
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                action2,
                                action5
                        )
                        ,action3
                )
        );
//robot moves to (action1) while moving slides up
        //robot then stops and lowers slides
        /*
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                action1,
                                action2
                        ),
                        action3
                )
        );
         */

        //robot moves to (action1) while moving slides up
        //robot then moves back to original position and lowers slides
        /*
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                action1,
                                action2
                        ),
                         new ParallelAction(
                                 action3,
                                 action4

                         )
                )
        );*/

    }
}
