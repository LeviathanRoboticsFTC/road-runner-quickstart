package org.firstinspires.ftc.teamcode.roadrunnerTest;

import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.threshold;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class VisionDetectionPipline2 implements VisionProcessor {
    double centerY = -1;
    int zone=0;

    double threshold = 1;

    List<MatOfPoint> Contours=new ArrayList<>();
    Moments m;
    Mat mat = new Mat();
    Mat hierarchy = new Mat();
    Mat kernal = new Mat();
    Mat kernal2 = new Mat();
    Mat Morph = new Mat();
    Mat Morph2 = new Mat();
    Mat ISOHSV = new Mat();
    Mat HSV = new Mat();
    Scalar lower;
    Scalar upper;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //way to represent the 3 values
       lower = new Scalar(5,5,5);
       upper = new Scalar(8,8,8);

       //Way to determine the number of pixels that we try to fill in
       kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15,15));
       kernal2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15,15));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //Converts to HSV (better color space)
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);

        Core.inRange(HSV,lower,upper,ISOHSV);

        //adds pixels and takes them away (fills holes)
        Imgproc.morphologyEx(ISOHSV, Morph, MORPH_OPEN, kernal);
        Imgproc.morphologyEx(Morph, Morph2, MORPH_CLOSE, kernal2);
        Imgproc.findContours(Morph2, Contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int contourIndex=-1;
        double contourArea=-1;

        for (int i = 0; i < Contours.size(); i++){

            double area = Imgproc.contourArea(Contours.get(i));

            if(area> contourArea && area > threshold){
                //sets index to the biggest area and sets biggest area to "area"
                contourArea = area;
                contourIndex = i;
            }
        }
        //checks if you see any shapes
        if(Contours.size() > 0&& contourIndex>=0){
            mat = frame.clone();
            //takes values and puts them back into the original image
            Imgproc.drawContours(mat, Contours, contourIndex, new Scalar(255, 255, 255), -1);

            m = Imgproc.moments(Contours.get(contourIndex));
            if(m.m00!=0){
                centerY = m.m01/m.m00;
            }else{
                centerY = -1;
            }

        }else{//if not sets centerY to non-existent
            mat = frame;
            centerY = -1;
        }
        //set zone based on where the centerY is
        if(centerY<0){
            zone = 0;
        }
        else if(centerY<20){
            zone = 1;
        }
        else {
            zone = 2;
        }

        return mat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
