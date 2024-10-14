package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleMarkingPipeline extends OpenCvPipeline {

    Point[] roiPoints = new Point[] {
            new Point(580, 1080),
            new Point(785, 540),
            new Point(905, 480),
            new Point(1102, 467),
            new Point(1275, 468),
            new Point(1329, 475),
            new Point(1501, 508),
            new Point(1563, 525),
            new Point(1747, 593),
            new Point(1920, 677),
            new Point(1920, 1080)
    };

    // Field reference values
    private final double img_ref1_y = 1080;
    private final double img_ref2_y = 623;
    private final double img_ref3_y = 434;

    private final double irl_ref1_y = 0;
    private final double irl_ref2_y = 10.5;
    private final double irl_ref3_y = 24;

    private final double irl_ref1_x = 6;
    private final double irl_ref2_x = 15.5;
    private final double irl_ref3_x = 19.5;

    private final double field_x_offset = 3.2;
    private final double field_y_offset = 6;

    MatOfPoint ROI = new MatOfPoint(roiPoints);

    //Camera parameters
//    public static final double fx = 463.94,fy = 462.24;
//    public static final double camX = 0,camY = 0;

//    int CAMERA_HEIGHT = 480;
//    int CAMERA_WIDTH = 640;

    int CAMERA_HEIGHT = 960;
    int CAMERA_WIDTH = 1280;

    public enum MaskColor {
        YELLOW,
        BLUE,
        RED
    }

    public enum GripperOrientation{
        VERTICAL,
        HORIZONTAL
    }

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    double cX,cY;

    public static class AnalyzedStone{
        MatOfPoint contour;
        public double area;
        public Point centroid;
        public Point field_pos;
        public GripperOrientation orientation;
    }

    ArrayList<SampleMarkingPipeline.AnalyzedStone> internalRedSampleList = new ArrayList<>();
    ArrayList<SampleMarkingPipeline.AnalyzedStone> internalYellowSampleList = new ArrayList<>();
    ArrayList<SampleMarkingPipeline.AnalyzedStone> internalBlueSampleList = new ArrayList<>();

    public ArrayList<SampleMarkingPipeline.AnalyzedStone> RedSampleList = new ArrayList<>();
    public ArrayList<SampleMarkingPipeline.AnalyzedStone> YellowSampleList = new ArrayList<>();
    public ArrayList<SampleMarkingPipeline.AnalyzedStone> BlueSampleList = new ArrayList<>();

    List<MatOfPoint> yellowContours;
    List<MatOfPoint> redContours;
    List<MatOfPoint> blueContours;

    Mat redMask;
    Mat Mask = new Mat();
    Mat hsvFrame = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        RedSampleList.clear();
//        YellowSampleList.clear();
//        BlueSampleList.clear();

        redMask = preprocessFrame(input, ColorConst.lowRed, ColorConst.highRed);
//        Mat yellowMask = preprocessFrame(input, ColorConst.lowYellow, ColorConst.highYellow);
//        Mat blueMask = preprocessFrame(input, ColorConst.lowBlue, ColorConst.highBlue);

        // Find contours of the detected regions
        redContours = findContours(redMask);
//        yellowContours = findContours(yellowMask);
//        blueContours = findContours(blueMask);

        //Update contours
        AnalyzeContour(redContours,internalRedSampleList);
//        AnalyzeContour(blueContours,internalBlueSampleList);
//        AnalyzeContour(yellowContours,internalYellowSampleList);

        RedSampleList = new ArrayList<>(internalRedSampleList);
//        YellowSampleList = new ArrayList<>(internalYellowSampleList);
//        BlueSampleList = new ArrayList<>(internalBlueSampleList);

        // Display all contours
//        drawAllContours(input, YellowSampleList, SampleDetectionPipeline.MaskColor.YELLOW);
//        drawAllContours(input, BlueSampleList, SampleDetectionPipeline.MaskColor.BLUE);
        drawAllContours(input, RedSampleList, SampleDetectionPipeline.MaskColor.RED);

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

    public List<AnalyzedStone> getSampleList(MaskColor color){
        switch(color){
            case RED:
                return RedSampleList;
            case BLUE:
                return BlueSampleList;
            case YELLOW:
                return YellowSampleList;
            default:
                return null;
        }
    }

    private void AnalyzeContour(List<MatOfPoint> contours,ArrayList<SampleMarkingPipeline.AnalyzedStone> samplelist){
        samplelist.clear();
        for (MatOfPoint contour: contours){
            AnalyzedStone Sample = new AnalyzedStone();
            Moments moments = Imgproc.moments(contour);
            int area = (int) moments.get_m00();
            Point cent = new Point(moments.get_m10() / area,moments.get_m01() / area);
            boolean IsInside = Imgproc.pointPolygonTest(new MatOfPoint2f(ROI.toArray()),cent,false)>0;

            if (area>100 && IsInside){
                Sample.contour = contour;
                Sample.centroid = cent;
                Sample.area = area;
                Sample.field_pos = locate_point(Sample.centroid);
                Sample.orientation = gripper_orient(contour);

                samplelist.add(Sample);
            }
        }
    }

    private void drawAllContours(Mat input, ArrayList<SampleMarkingPipeline.AnalyzedStone> samplelist, SampleDetectionPipeline.MaskColor maskColor) {
        String obj_color = (maskColor == SampleDetectionPipeline.MaskColor.RED) ? "R" :
                (maskColor == SampleDetectionPipeline.MaskColor.BLUE) ? "B" :
                        (maskColor == SampleDetectionPipeline.MaskColor.YELLOW) ? "Y" : "";

        for (AnalyzedStone sample : samplelist) {
            int area = (int) sample.area;
            cX = sample.centroid.x;
            cY = sample.centroid.y;

            // Draw a dot at the centroid
            Imgproc.circle(input, sample.centroid, 3, new Scalar(0, 255, 0), -1);
            String label = "(" + String.format("%.2f",sample.field_pos.x) + ", " + String.format("%.2f",sample.field_pos.y) + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 1);
        }
    }

    private Point locate_point(Point centroid){
        double x = 960 - centroid.x;
        double y = centroid.y;

        double cross_r = (img_ref1_y - y) * (img_ref2_y - img_ref3_y) * (irl_ref3_y - irl_ref1_y)
                / (img_ref1_y - img_ref3_y) / (img_ref2_y - y) / (irl_ref3_y - irl_ref2_y);

        double field_x = (irl_ref2_y * cross_r - irl_ref1_y) / (cross_r - 1);
        double y_width_x = field_x * (irl_ref2_x - irl_ref1_x) / (irl_ref2_y - irl_ref1_y) + irl_ref1_x;
        double field_y = x / 960.0 * y_width_x;

        return new Point(field_x, field_y);
    }

    private GripperOrientation gripper_orient(MatOfPoint contour){
        Rect x = Imgproc.boundingRect(contour);
        if(x.width>x.height) return GripperOrientation.VERTICAL;
        else return GripperOrientation.HORIZONTAL;
    }
}
