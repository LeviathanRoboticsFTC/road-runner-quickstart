package org.firstinspires.ftc.teamcode.roadrunnerTest;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Pose2d;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp

public class RoadrunnerApriltagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        AprilTagProcessor myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(276.618, 276.618, 318.459, 283.057)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(myAprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();



// Build 3 trajectories for 3 different Apriltag
        Action TrajectoryAction1  = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0, 23))
                .build();
        Action TrajectoryAction2  = drive.actionBuilder(drive.pose)
                .lineToXLinearHeading(-23,0)
                .build();
        Action TrajectoryAction3  = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(0, -23))
                .build();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            if(myAprilTagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = myAprilTagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }
            ArrayList<AprilTagDetection> myAprilTagDetections;  // list of all detections
            AprilTagDetection myAprilTagDetection;         // current detection in for() loop
            int myAprilTagIdCode;                           // ID code of current detection, in for() loop

// Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata != null) {

                    //Get current tag ID
                    myAprilTagIdCode = detection.id;

                    //Update the tag details
                    telemetry.addData("x", detection.ftcPose.x);
                    telemetry.addData("y", detection.ftcPose.y);
                    telemetry.addData("z", detection.ftcPose.z);
                    telemetry.addData("roll", detection.ftcPose.roll);
                    telemetry.addData("pitch", detection.ftcPose.pitch);
                    telemetry.addData("yaw", detection.ftcPose.yaw);
                    telemetry.addData("bearing", detection.ftcPose.bearing);
                    telemetry.addData("range", detection.ftcPose.range );

                    /* Run trajectories based on AprilTag ID.
                    Tag ID 1: Strafe left 1 tile (23 inches)
                    Tag ID 2: Go backwards 1 tile
                    Tag ID 3: Strafe right 1 tile (23 inches)
                    Invalid ID: Show "Unknown Detection"
                     */
                    switch (myAprilTagIdCode){
                        case 1:
                            Actions.runBlocking(
                                    new SequentialAction(
                                            TrajectoryAction1,
                                            new Action() {
                                                @Override
                                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                                    telemetry.addData("Detection", "1");
                                                    telemetry.update();
                                                    return false;
                                                }
                                            }
                                    )
                            );

                            break;
                        case 2:
                            Actions.runBlocking(
                                    new SequentialAction(
                                            TrajectoryAction2,
                                            new Action() {
                                                @Override
                                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                                    telemetry.addData("Detection", "2");
                                                    telemetry.update();
                                                    return false;
                                                }
                                            }
                                    )
                            );
                            break;
                        case 3:
                            Actions.runBlocking(
                                    new SequentialAction(
                                            TrajectoryAction3,
                                            new Action() {
                                                @Override
                                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                                    telemetry.addData("Detection", "3");
                                                    telemetry.update();
                                                    return false;
                                                }
                                            }
                                    )
                            );
                            break;

                    }


                }
                else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }

// Cycle through through the list and process each AprilTag.


            telemetry.update();
        }

    }
}
