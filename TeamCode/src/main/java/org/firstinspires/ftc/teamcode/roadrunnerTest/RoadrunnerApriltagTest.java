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
    private int targetTagID = 0;

    private double xyScaling = 1.00;
    private double rangeCorrection = 0; //inches
    private double tagX = 24;//62.34;  //inches
    private double tagY1 = 5;//41.65; //blue left tagID = 1
    private double tagY2 = 0;//35.60; //blue center tagID = 2
    private double tagY3 = -5;//29.61; //blue right tagID = 3
    private double tagY4 = -29.61; //red left tagID = 4
    private double tagY5 = -35.60; //red center tagID = 5
    private double tagY6 = -62.34; //red right tagID = 6
    private double  dCamera = 5; //distance from center of robot to camera

    private double[] getRobotXY(AprilTagDetection tag) {
        double[] robotXY = new double[8];
        double tagY = 0; //inches

        if (tag.id==1){
            tagY = tagY1;}
        else if (tag.id==2) {
            tagY = tagY2;}
        else if (tag.id==3) {
            tagY = tagY3;}
        else if (tag.id==4) {
            tagY = tagY4;}
        else if (tag.id==5) {
            tagY = tagY5;}
        else if (tag.id==6) {
            tagY = tagY6;}

        // get range, bearing, and yaw from april tag
        double rangeAT = tag.ftcPose.range + rangeCorrection;
        double bearingAT = Math.toRadians(tag.ftcPose.bearing);
        double yawAT = Math.toRadians(tag.ftcPose.yaw);

        //calculating deltaX and deltaY of the tag relative to the camera
        double camTagX = rangeAT * Math.cos(bearingAT - yawAT);
        double camTagY = rangeAT * Math.sin(bearingAT - yawAT);

        //calculating deltaX and deltaY of the camera relative to the center of robot
        double camX = dCamera * Math.cos(-yawAT);
        double camY = dCamera * Math.sin(-yawAT);

        //calculating relative X and relative Y of the robot to the tag
        robotXY[0] = xyScaling*(tagX - camTagX - camX);
        robotXY[1] = xyScaling*(tagY - camTagY - camY);
        robotXY[2] = tagX;
        robotXY[3] = camTagX;
        robotXY[4] = camX;
        robotXY[5] = tagY;
        robotXY[6] = camTagY;
        robotXY[7] = camY;

        return robotXY;
    }
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

        if (isStopRequested()) return;

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
                    double[] robotXYthisTag = getRobotXY(detection);

                    myAprilTagIdCode = detection.id;
                    if (myAprilTagIdCode == 1){
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
                    }else if(myAprilTagIdCode == 2){
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
                    }else{
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryAction3,

                                        new Action() {
                                            @Override
                                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                                telemetry.addData("Detection", "3 or none");
                                                telemetry.update();
                                                return false;
                                            }
                                        }

                                )
                        );
                    }

                    telemetry.addData("yaw", detection.ftcPose.yaw);
                    telemetry.addData("bearing", detection.ftcPose.bearing);
                    telemetry.addData("range", detection.ftcPose.range );
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
