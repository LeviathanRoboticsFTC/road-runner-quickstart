package org.firstinspires.ftc.teamcode.CenterStageSimulation;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Pipeline implements VisionProcessor {
    int ZoneNum;
    Mat HSV = new Mat();
    Rect rect1;
    Rect rect2;
    Rect rect3;
    Mat zone1 = new Mat();
    Mat zone2 = new Mat();
    Mat zone3 = new Mat();
    //defines the lower and upper boundary of the hue value
    //hue value on colorizer.org divide by 2
    double HLower = 90;
    double HUpper = 127;
    //defines the lower boundaries of saturation value
    double SLower = 30;
    // returns the team prop
    public int getZone(){
        return ZoneNum;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // This places the rectangles where the team prop should be
        rect1 = new Rect(new Point(10, 320), new Point(110, 480));
        rect2 = new Rect(new Point(260, 320), new Point(360, 480));
        rect3 = new Rect(new Point(540, 320), new Point(640, 480));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //keeps track of amount of pixel in each zone
        double counter1=0;
        double counter2=0;
        double counter3=0;

        Imgproc.rectangle(frame, rect1, new Scalar(255,0,0));
        Imgproc.rectangle(frame, rect2, new Scalar(255,0,0));
        Imgproc.rectangle(frame, rect3, new Scalar(255,0,0));

        //Converts to HSV (better color space)
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);

        //cuts out specific sections of the image
        zone1=HSV.submat(rect1);
        zone2=HSV.submat(rect2);
        zone3=HSV.submat(rect3);

        //iterates through matrix (goes through the pixels)
        for(int i=0; i<zone1.rows(); i++)
        {
            for (int j=0; j<zone1.cols(); j++)
            {
                //checks if pixel valid
                if(zone1.get(i, j)[0] > HLower && zone1.get(i, j)[0] < HUpper && zone1.get(i, j)[1] > SLower)
                {
                    //increments counter
                    counter1++;
                }
            }
        }
        for(int i=0; i<zone2.rows(); i++)
        {
            for (int j=0; j<zone2.cols(); j++)
            {
                //checks if pixel valid
                if(zone2.get(i, j)[0] > HLower && zone2.get(i, j)[0] < HUpper && zone2.get(i, j)[1] > SLower)
                {
                    //increments counter
                    counter2++;
                }
            }
        }
        for(int i=0; i<zone3.rows(); i++)
        {
            for (int j=0; j<zone3.cols(); j++)
            {
                //checks if pixel valid
                if(zone3.get(i, j)[0] > HLower && zone3.get(i, j)[0] < HUpper && zone3.get(i, j)[1] > SLower)
                {
                    //increments counter
                    counter3++;
                }
            }
        }

        //finds the largest counter
        double largestCounter = Math.max(Math.max(counter1, counter2), counter3);
        if (counter1 == largestCounter){
            ZoneNum = 1;
        }
        else if (counter2 == largestCounter) {
            ZoneNum = 2;
        }
        else if (counter3 == largestCounter) {
            ZoneNum = 3;
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
