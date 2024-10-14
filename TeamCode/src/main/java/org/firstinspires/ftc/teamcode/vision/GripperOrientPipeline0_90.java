package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GripperOrientPipeline0_90 extends OpenCvPipeline {

    Mat Mask = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    MatOfPoint2f l_contourf = new MatOfPoint2f();
    Mat hsvFrame = new Mat();
    MatOfPoint l_contour = new MatOfPoint();
    Mat ColorMask = new Mat();

    public enum GripperAngle{
        ANGLE_0,
        ANGLE_90
    }
    GripperAngle angle = GripperAngle.ANGLE_0;

    @Override
    public Mat processFrame(Mat input) {
        ColorMask = preprocessFrame(input,ColorConst.lowRed,ColorConst.highRed);
        contours = findContours(ColorMask);
        Imgproc.drawContours(input,contours,-1,new Scalar(0,0,0),3);
        l_contour = findLargestContour(contours);
        l_contourf = new MatOfPoint2f(l_contour.toArray());
//        String text = String.format("%.2f",get_orientation(l_contourf));
//        Imgproc.putText(input,text, new Point(10, 30), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 255, 0), 1);
        return input;
    }

    private Mat preprocessFrame(Mat frame, Scalar low, Scalar high) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsvFrame, low, high, Mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_CLOSE, kernel);

        return Mask;
    }
    private List<MatOfPoint> findContours(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }
    public GripperAngle getAngle(MatOfPoint2f contour){
        return GripperAngle.ANGLE_0;
    }
}
