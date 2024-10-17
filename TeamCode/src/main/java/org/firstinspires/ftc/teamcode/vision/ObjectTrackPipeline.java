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

public class ObjectTrackPipeline extends OpenCvPipeline {

    public volatile boolean error = false;
    public volatile Exception debug;
    private final Object sync = new Object();

    //Loop counters
    private int loopCounter = 0;
    private int pLoopCounter = 0;


    public enum MaskColor {
        YELLOW,
        BLUE,
        RED
    }

    public enum GripperOrientation{
        VERTICAL,
        HORIZONTAL
    }

    public static class AnalyzedStone{
        MatOfPoint contour;
        public double area;
        public Point centroid;
        public Point field_pos;
        public GripperOrientation orientation;
    }

    ArrayList<AnalyzedStone> internalRedSampleList = new ArrayList<>();
    ArrayList<AnalyzedStone> internalYellowSampleList = new ArrayList<>();
    ArrayList<AnalyzedStone> internalBlueSampleList = new ArrayList<>();

    volatile ArrayList<AnalyzedStone> RedSampleList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> YellowSampleList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> BlueSampleList = new ArrayList<>();
    List<MatOfPoint> yellowContours;
    List<MatOfPoint> redContours;
    List<MatOfPoint> blueContours;

    Mat redMask = new Mat();
    Mat Mask = new Mat();
    Mat hsvFrame = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        try{
            RedSampleList.clear();

            redMask = preprocessFrame(input, VisionConst.lowRed, VisionConst.highRed);

            redContours = findContours(redMask);

            synchronized (sync) {
                AnalyzeContour(redContours, internalRedSampleList);
                RedSampleList = new ArrayList<>(internalRedSampleList);
                drawAllContours(input, RedSampleList, MaskColor.RED);
            }
        }
        catch (Exception e){
            debug = e;
            error = true;
        }
        return input;
    }

    public List<AnalyzedStone> getSampleList(){
        synchronized (sync){
            return RedSampleList;
        }
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
    private void AnalyzeContour(List<MatOfPoint> contours,ArrayList<AnalyzedStone> samplelist) {
        samplelist.clear();
        for (MatOfPoint contour : contours) {
            AnalyzedStone Sample = new AnalyzedStone();
            Moments moments = Imgproc.moments(contour);
            int area = (int) moments.get_m00();
            Point cent = new Point(moments.get_m10() / area, moments.get_m01() / area);
            boolean IsInside = Imgproc.pointPolygonTest(new MatOfPoint2f(VisionConst.ROI.toArray()), cent, false) > 0;

            if (area > 100 && IsInside) {
                Sample.contour = contour;
                Sample.centroid = cent;
                Sample.area = area;
                Sample.field_pos = locate_point(Sample.centroid);
                Sample.orientation = gripper_orient(contour);

                samplelist.add(Sample);
            }
        }
    }
    private Point locate_point(Point centroid){
        double x = 960 - centroid.x;
        double y = centroid.y;

        double cross_r = (VisionConst.img_ref1_y - y) * (VisionConst.img_ref2_y - VisionConst.img_ref3_y) * (VisionConst.irl_ref3_y - VisionConst.irl_ref1_y)
                / (VisionConst.img_ref1_y - VisionConst.img_ref3_y) / (VisionConst.img_ref2_y - y) / (VisionConst.irl_ref3_y - VisionConst.irl_ref2_y);

        double field_x = (VisionConst.irl_ref2_y * cross_r - VisionConst.irl_ref1_y) / (cross_r - 1);
        double y_width_x = field_x * (VisionConst.irl_ref2_x - VisionConst.irl_ref1_x) / (VisionConst.irl_ref2_y - VisionConst.irl_ref1_y) + VisionConst.irl_ref1_x;
        double field_y = x / 960.0 * y_width_x;

        return new Point(field_x, field_y);
    }
    private GripperOrientation gripper_orient(MatOfPoint contour){
        Rect x = Imgproc.boundingRect(contour);
        if(x.width>x.height) return GripperOrientation.VERTICAL;
        else return GripperOrientation.HORIZONTAL;
    }
    private void drawAllContours(Mat input, ArrayList<AnalyzedStone> samplelist, MaskColor maskColor) {
        String obj_color = (maskColor == MaskColor.RED) ? "R" :
                (maskColor == MaskColor.BLUE) ? "B" :
                        (maskColor == MaskColor.YELLOW) ? "Y" : "";

        for (AnalyzedStone sample : samplelist) {
            // Draw a dot at the centroid
            Imgproc.circle(input, sample.centroid, 3, new Scalar(0, 255, 0), -1);
            String label = "(" + String.format("%.2f",sample.field_pos.x) + ", " + String.format("%.2f",sample.field_pos.y) + ")";
            Imgproc.putText(input, label, new Point(sample.centroid.x + 10, sample.centroid.y + 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 1);
        }
    }
}
