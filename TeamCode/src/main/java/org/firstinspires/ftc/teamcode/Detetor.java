//package org.firstinspires.ftc.teamcode;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//public class Detetor extends OpenCvPipeline {
//    Mat mat = new Mat();
//
//    @Override
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar(23,50,70);
//        Scalar highHSV = new Scalar(32,255,255);
//
//        Core.inRange(mat,lowHSV,highHSV,mat);
//    }
//}
