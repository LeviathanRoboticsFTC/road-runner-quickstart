package org.firstinspires.ftc.teamcode.roadrunnerTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.officialLeviathanCode.programmingBoard.ProgrammingBoardV1;

@TeleOp
public class TrajectoryTest extends LinearOpMode{
    @Override
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ProgrammingBoardV1 board = new ProgrammingBoardV1();

        Action TrajectoryAction1  = drive.actionBuilder(drive.pose)
                .lineToX(10)
                .build();
        Action TrajectoryAction2 = drive.actionBuilder(new Pose2d(15,20,0))
                .lineToX(10)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while(!isStopRequested() && opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction2, // Example of a drive action

                            // This action and the following action do the same thing
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    telemetry.addLine("Action!");
                                    telemetry.update();
                                    return false;
                                }
                            },
                            // Only that this action uses a Lambda expression to reduce complexity
                            (telemetryPacket) -> {
                                telemetry.addLine("Action!");
                                telemetry.update();
                                return false; // Returning true causes the action to run again, returning false causes it to cease
                            },
                            /**
                             * new ParallelAction( // several actions being run in parallel
                             TrajectoryAction2, // Run second trajectory
                             (telemetryPacket) -> { // Run some action
                             board.drivePowerFoward(0.6);
                             return false;
                             }
                             ),
                             */
                            drive.actionBuilder(new Pose2d(10,0,Math.toRadians(90))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                    .splineTo(new Vector2d(23, 0), 0)
                                    .build()

                    )
            );

        }




    }
}
