package org.firstinspires.ftc.teamcode.roadrunnerTest;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class VisionDetectionTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        VisionDetectionPipeline pipeline = new VisionDetectionPipeline();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(pipeline)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }
}
