package org.firstinspires.ftc.teamcode.roadrunnerTest;
import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
@TeleOp
public class RunToAprilTag extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
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
        boolean taskCompleted = false;
        waitForStart();
        while (!isStopRequested() && opModeIsActive() && !taskCompleted) {
            ArrayList<AprilTagDetection> myAprilTagDetections;  // list of all detections
            AprilTagDetection myAprilTagDetection;         // current detection in for() loop
            int myAprilTagIdCode;                           // ID code of current detection
            // Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.metadata != null) {
                    //Get current tag ID
                    myAprilTagIdCode = detection.id;
                    //Update the tag details
                    telemetry.addData("x", 2*detection.ftcPose.x);
                    telemetry.addData("y", 2*detection.ftcPose.y);
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
                        double tagX = 2*detection.ftcPose.x;
                        double tagY = 2*detection.ftcPose.y-1;
                        double turnYaw = Math.toRadians(detection.ftcPose.yaw);

                        Action TurnToAprilTag = drive.actionBuilder(drive.pose)
                                .turnTo(-detection.ftcPose.yaw)
                                .endTrajectory()
                                .build();
                        Action MoveToTag = drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(0, detection.ftcPose.x))
                                .endTrajectory()
                                .lineToXLinearHeading(detection.ftcPose.y - 2, 0)
                                .endTrajectory()
                                .build();
                        Action SplineToTag = drive.actionBuilder(new Pose2d(0,0,0))
                                .splineToConstantHeading(new Vector2d(tagY, -tagX), turnYaw)
                                .build();


                        Actions.runBlocking(
                                new SequentialAction(SplineToTag)
                        );


                        taskCompleted = true;



                        /*
                        if (detection.ftcPose.yaw > 0.5) {

                            // turn the bot to be parallel with the AprilTag
                            Actions.runBlocking(
                                    new SequentialAction(TurnToAprilTag)
                            );
                        } else {
                            // move the bot to 2 inches before the AprilTag
                            Actions.runBlocking(
                                    new SequentialAction(MoveToTag)
                            );
                            taskCompleted = true;
                            telemetry.addData("Trajectory completed. Robot moved to ID: ", detection.id );
                        }

                         */
                    }
                }
                else {
                    telemetry.addData("Unknown AprilTag ID:" , detection.id);
                }
            }
            telemetry.update();
        }
    }
}