package org.firstinspires.ftc.teamcode.roadrunnerTest;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp

public class RoadrunnerApriltagTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        AprilTagProcessor myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(647.83, 647.83, 331.263, 220.186)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(myAprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        boolean taskCompleted = false;

        waitForStart();
        Action KeepTurning = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(5))
                .build();


        while (!isStopRequested() && opModeIsActive() && !taskCompleted) {


            ArrayList<AprilTagDetection> myAprilTagDetections;  // list of all detections
            int myAprilTagIdCode;                           // ID code of current detection

            // Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            while(myAprilTagDetections.size() == 0){
                Actions.runBlocking(
                        new SequentialAction(KeepTurning)
                );
                myAprilTagDetections = myAprilTagProcessor.getDetections();

            }


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
                        telemetry.addData("range", detection.ftcPose.range);


                    /*
                    Find AprilTag 1 and move the bot
                     */

                        if (myAprilTagIdCode == 1) {

                        /* Run trajectories based on AprilTag ID.
                    Trajectory 1: Turn counter clock to a degree of yaw
                    Trajectory 2: Strafe to left X
                    Trajectory 3: move forward to Y - 2
                     */
                            double xTag = detection.ftcPose.x;
                            double yTag = detection.ftcPose.y;
                            double yawTag = detection.ftcPose.yaw;
                            double bearingTag = detection.ftcPose.bearing;

                        /*


                        Action RunToAprilTag = drive.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeToLinearHeading(new Vector2d(yTag-2,xTag),Math.toRadians(yawTag) )
                                .build();


                        Actions.runBlocking(
                                new SequentialAction(RunToAprilTag)
                        );

                         */


                            //taskCompleted = true;


                            telemetry.addData("Trajectory completed. Robot moved to ID: ", detection.id);
                            break;
                        }
                    } else {
                        telemetry.addData("IDs Found", 0);

                    }


                }

            telemetry.update();
        }
    }
}