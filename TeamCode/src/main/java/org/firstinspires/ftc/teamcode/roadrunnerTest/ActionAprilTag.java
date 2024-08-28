package org.firstinspires.ftc.teamcode.roadrunnerTest;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class ActionAprilTag extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        //Action for turning to the tag
        MecanumDrive.TurnUntillAprillTag TurnTillTag = drive.new TurnUntillAprillTag(

        );

        waitForStart();
        Actions.runBlocking(
                TurnTillTag
        );

        double xTag = TurnTillTag.getDetection().ftcPose.x;
        double yTag = TurnTillTag.getDetection().ftcPose.y;
        double yawTag = TurnTillTag.getDetection().ftcPose.yaw;
        double bearingTag = TurnTillTag.getDetection().ftcPose.bearing;


        telemetry.addData("yawTag", yawTag);
            telemetry.addData("xTag", xTag);
            telemetry.addData("yTag", yTag);
            telemetry.update();


            Action runToTag = drive.actionBuilder(new Pose2d(0, 0, 0))

                    .turnTo(Math.toRadians(yawTag))
                    .strafeTo(new Vector2d(yTag, -xTag))
                    .waitSeconds(5)
                    .build();
            Actions.runBlocking(
                    runToTag
            );




    }
}
