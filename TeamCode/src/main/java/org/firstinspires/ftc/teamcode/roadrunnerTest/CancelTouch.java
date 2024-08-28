package org.firstinspires.ftc.teamcode.roadrunnerTest;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SlidesBoard;

@TeleOp
public class CancelTouch extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touch");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        SlidesBoard Slides = new SlidesBoard();
        Slides.init(hardwareMap);
        Action tempAction = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(23,23))
                .build();
        // make tempAction cancellable
        //maks slides cancellabld
        SlidesBoard.SlidesToPosition tempaction2 = Slides.new SlidesToPosition(6800, telemetry);
        SlidesBoard.SlidesToPosition action3 = Slides.new SlidesToPosition(0, telemetry);
        MecanumDrive.FollowTrajectoryTouchCancel action2 = drive.new FollowTrajectoryTouchCancel(tempAction);
        SlidesBoard.CancellableSlides CancleSlides = Slides.new CancellableSlides(tempaction2, gamepad1);
        SlidesBoard.TouchCancleSlides CancelTouchSlides = Slides.new TouchCancleSlides(tempaction2, touchSensor);
        waitForStart();

        Actions.runBlocking(
                CancelTouchSlides
        );
    }
}
