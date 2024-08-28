package org.firstinspires.ftc.teamcode.CenterStageSimulation;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SlidesBoard;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous

public class VisionTest extends LinearOpMode {
int zone;
    @Override
    public void runOpMode() throws InterruptedException {
        Pipeline pipeline = new Pipeline();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        SlidesBoard slides = new SlidesBoard();
        slides.init(hardwareMap);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(pipeline)
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        while (isStarted()!=true){
            zone = pipeline.getZone();
            telemetry.addData("Zone",zone);
            telemetry.update();
        }
        //create movement action but don't define it
        Action movement1;
        //slides go up all the way
        Action slides1 = slides.new SlidesToPosition(6800, telemetry);

        // define movement1 based on the zone number
        //forward is +x, left is +y, camera direction
        if(zone == 1)
        {
            //strafe left by a block if zone1
            movement1 = drive.actionBuilder(new Pose2d(0,0,0))
                    .strafeTo(new Vector2d(0, 23))
                    .build();
        }else if(zone == 2){
            //move forward by a block if zone2
            movement1 = drive.actionBuilder(new Pose2d(0,0,0))
                    .strafeTo(new Vector2d(23, 0))
                    .build();
        }else{
            //strafe right by a block if zone3
            movement1 = drive.actionBuilder(new Pose2d(0,0,0))
                    .strafeTo(new Vector2d(0, -23))
                    .build();
        }
        /*Actions.runBlocking(
                new ParallelAction(
                        movement1,
                        slides1
                )
        );*/
        Actions.runBlocking(movement1);

        ArrayList<AprilTagDetection> myAprilTagDetections;  // list of all detections
        myAprilTagDetections = tagProcessor.getDetections();
        for (AprilTagDetection detection : myAprilTagDetections) {

            if (detection.metadata != null) {
                int myAprilTagIdCode = detection.id;

                //Update the tag details
                telemetry.addData("x", detection.ftcPose.x);
                telemetry.addData("y", detection.ftcPose.y);
                telemetry.addData("z", detection.ftcPose.z);
                telemetry.addData("roll", detection.ftcPose.roll);
                telemetry.addData("pitch", detection.ftcPose.pitch);
                telemetry.addData("yaw", detection.ftcPose.yaw);
                telemetry.addData("bearing", detection.ftcPose.bearing);
                telemetry.addData("range", detection.ftcPose.range);


                if (myAprilTagIdCode == zone) {
                    Log.d("Y Distance", String.valueOf(detection.ftcPose.y));

                    Action runToTag = drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x + detection.ftcPose.y, drive.pose.position.y - detection.ftcPose.x))
                            .build();
                    Actions.runBlocking(
                            runToTag
                    );
                }
            }
        }

    }
}
