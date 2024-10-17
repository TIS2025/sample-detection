package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class VisionConst {
    //Color thresholds
    public static Scalar highRed = new Scalar(130, 255, 255);
    public static Scalar lowRed = new Scalar(120, 70, 50);
    public static Scalar highBlue = new Scalar(30, 255, 255);
    public static Scalar lowBlue = new Scalar(0, 50, 30);//70,50
    public static Scalar highYellow = new Scalar(100, 255, 255);
    public static Scalar lowYellow = new Scalar(90, 70, 50);

    //Camera Streaming Resolution
    public static int CAM_WIDTH = 1280;
    public static int CAM_HEIGHT = 720;

    //Field Const
    public static Point[] roiPoints = new Point[] {
            new Point(257, 720),
            new Point(464, 272),
            new Point(816, 272),
            new Point(1023, 720),
    };

    public static double img_ref1_y = 1080;
    public static double img_ref2_y = 623;
    public static double img_ref3_y = 434;

    public static double irl_ref1_y = 0;
    public static double irl_ref2_y = 10.5;
    public static double irl_ref3_y = 24;

    public static double irl_ref1_x = 6;
    public static double irl_ref2_x = 15.5;
    public static double irl_ref3_x = 19.5;

    public static double field_x_offset = 3.2;
    public static double field_y_offset = 6;

    public static MatOfPoint ROI = new MatOfPoint(roiPoints);

}
