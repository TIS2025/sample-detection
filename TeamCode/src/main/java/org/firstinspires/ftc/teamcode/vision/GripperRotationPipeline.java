package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class GripperRotationPipeline extends OpenCvPipeline {

    Mat hsvFrame = new Mat();
    Mat ColorMask = new Mat();
    Mat Mask = new Mat();
    RotatedRect x = new RotatedRect();
    List<MatOfPoint> contours = new ArrayList<>();
    MatOfPoint l_contour = new MatOfPoint();
    MatOfPoint2f l_contourf = new MatOfPoint2f();
    Point[] vertices = new Point[4];
    Point tL,tR,bL,bR;

    public double angle = 0;
    @Override
    public Mat processFrame(Mat input) {
        ColorMask = preprocessFrame(input,ColorConst.lowRed,ColorConst.highRed);
        contours = findContours(ColorMask);
        Imgproc.drawContours(input,contours,-1,new Scalar(0,0,0),3);
        l_contour = findLargestContour(contours);
        l_contourf = new MatOfPoint2f(l_contour.toArray());
        String text = String.format("%.2f",get_orientation(l_contourf));
        Imgproc.putText(input,text, new Point(10, 30), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 255, 0), 1);
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
    public double get_orientation(MatOfPoint2f contour){
        x = Imgproc.minAreaRect(contour);
        x.points(vertices);

        Arrays.sort(vertices, Comparator.comparingDouble(point -> point.y));
        tL = vertices[0];
        bR = vertices[3];

        Arrays.sort(vertices, Comparator.comparingDouble(point -> point.x));
        bL = vertices[0];
        tR = vertices[3];

        double w = calculateDistance(tL,tR);
        double h =calculateDistance(tL,bL);

        if(tL.y==tR.y) return 0;
        if(tL.y==bL.y) return 90;

        if(w>=h) angle = 90-Math.toDegrees(Math.atan((tL.y-bL.y)/(tL.x-bL.x)));
        else angle = 90-Math.toDegrees(Math.atan((tR.y-tL.y)/(tR.x-tL.x)));

        return angle;
    }
    public RotatedRect getRect(){
        return x;
    }
    public double getAngle(){
        return angle;
    }
    private double calculateDistance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }
}
