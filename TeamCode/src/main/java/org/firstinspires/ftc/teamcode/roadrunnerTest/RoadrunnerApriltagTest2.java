package org.firstinspires.ftc.teamcode.roadrunnerTest;

import android.util.Size;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp

public class RoadrunnerApriltagTest2 extends LinearOpMode {

    AprilTagProcessor myAprilTagProcessor;
    boolean taskCompleted = false;
    public void KeepTurning(MecanumDrive drive){

        drive.leftFront.setPower(-0.2);
        drive.leftBack.setPower(-0.2);
        drive.rightBack.setPower(0.2);
        drive.rightFront.setPower(0.2);

        ArrayList<AprilTagDetection> myAprilTagDetections;  // list of all detections
        myAprilTagDetections = myAprilTagProcessor.getDetections();
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







                if (myAprilTagIdCode == 1) {
                    double xTag = detection.ftcPose.x;
                    double yTag = detection.ftcPose.y;
                    double yawTag = detection.ftcPose.yaw;
                    double bearingTag = detection.ftcPose.bearing;

                    drive.leftFront.setPower(0);
                    drive.leftBack.setPower(0);
                    drive.rightBack.setPower(0);
                    drive.rightFront.setPower(0);
                    taskCompleted = true;
                    telemetry.addData("yawTag", yawTag);
                    telemetry.addData("xTag", xTag);
                    telemetry.addData("yTag", yTag);
                    telemetry.update();


                    Action runToTag = drive.actionBuilder(new Pose2d(0, 0, 0))

                            .turnTo(Math.toRadians(yawTag))
                            .strafeTo(new Vector2d(yTag, -xTag))
                            .waitSeconds(5)
                            //.lineToY(5)
                            .build();
                    Actions.runBlocking(
                            runToTag
                    );



                }
            }
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        myAprilTagProcessor = new AprilTagProcessor.Builder()
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

        //puts camera stream to dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        waitForStart();



        while (!isStopRequested() && opModeIsActive() && !taskCompleted) {
            KeepTurning(drive);
            telemetry.update();
        }

    }
}