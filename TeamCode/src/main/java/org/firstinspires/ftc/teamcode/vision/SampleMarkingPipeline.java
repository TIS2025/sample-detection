package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleMarkingPipeline extends OpenCvPipeline {

    //Color thresholds
    public static Scalar highRed = new Scalar(130, 255, 255);
    public static Scalar lowRed = new Scalar(120, 70, 50);
    public static Scalar highBlue = new Scalar(30, 255, 255);
    public static Scalar lowBlue = new Scalar(0, 50, 30);//70,50
    public static Scalar highYellow = new Scalar(100, 255, 255);
    public static Scalar lowYellow = new Scalar(90, 70, 50);

    //Camera parameters
    public static final double fx = 463.94,fy = 462.24;
    public static final double cX = 0,cY = 0;

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();


    public SampleMarkingPipeline(){
        cameraMatrix.put(0, 0,
                  fx, 0, cX,
                        0, fy, cY,
                        0, 0, 1);

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // If you have calibrated your camera and have these values, use them
        // Otherwise, you can assume zero distortion for simplicity
        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }
//    cameraMatrix.(0, 0,
//    fx, 0, cx,
//            0, fy, cy,
//            0, 0, 1);

    // Distortion coefficients (k1, k2, p1, p2, k3)
    // If you have calibrated your camera and have these values, use them
    // Otherwise, you can assume zero distortion for simplicity

    @Override
    public Mat processFrame(Mat input) {

//        Mat yellowMask = preprocessFrame(input, lowYellow, highYellow);
//        Mat redMask = preprocessFrame(input, lowRed, highRed);
//        Mat blueMask = preprocessFrame(input, lowBlue, highBlue);


        return input;
    }
}
