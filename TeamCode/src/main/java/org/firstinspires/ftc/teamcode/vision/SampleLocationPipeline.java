//package org.firstinspires.ftc.teamcode.vision;
//
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class SampleLocationPipeline extends OpenCvPipeline {
//
//    private final Mat kernel;
//    public List<Sample> sampleListRed;
//    public List<Sample> tempSampleListRed;
//
//    public static class Sample {
//        public MatOfPoint contour;
//        public Point centroid;
//        public double area;
//        public Point field_pos;
//    }
//
//    // ROI polygon
//    private final MatOfPoint ROI = new MatOfPoint(
//            new Point(580, 1080),
//            new Point(785, 540),
//            new Point(905, 480),
//            new Point(1102, 467),
//            new Point(1275, 468),
//            new Point(1329, 475),
//            new Point(1501, 508),
//            new Point(1563, 525),
//            new Point(1747, 593),
//            new Point(1920, 677),
//            new Point(1920, 1080)
//    );
//
//    // Field reference values
//    private final double img_ref1_y = 1080;
//    private final double img_ref2_y = 623;
//    private final double img_ref3_y = 434;
//
//    private final double irl_ref1_y = 0;
//    private final double irl_ref2_y = 10.5;
//    private final double irl_ref3_y = 24;
//
//    private final double irl_ref1_x = 6;
//    private final double irl_ref2_x = 15.5;
//    private final double irl_ref3_x = 19.5;
//
//    private final double field_x_offset = 3.2;
//    private final double field_y_offset = 6;
//
//    // Color constants
//    private Scalar lowRed1 = new Scalar(0, 80, 80);
//    private Scalar highRed1 = new Scalar(10, 255, 255);
//    private Scalar lowRed2 = new Scalar(170, 70, 70);
//    private Scalar highRed2 = new Scalar(180, 255, 255);
//    private Scalar lowBlue = new Scalar(0, 50, 30);
//    private Scalar highBlue = new Scalar(30, 255, 255);
//    private Scalar lowYellow = new Scalar(90, 90, 90);
//    private Scalar highYellow = new Scalar(105, 255, 255);
//
//    public SampleLocationPipeline() {
//        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        sampleListRed = new ArrayList<>();
//        tempSampleListRed = new ArrayList<>();
//    }
//
//    public void analyzeContour(List<MatOfPoint> contours) {
//        tempSampleListRed.clear();
//        for (MatOfPoint contour : contours) {
//            Moments moments = Imgproc.moments(contour);
//            int cx = (int) (moments.get_m10() / moments.get_m00());
//            int cy = (int) (moments.get_m01() / moments.get_m00());
//
//            if (moments.get_m00() > 400 && Imgproc.pointPolygonTest(new MatOfPoint2f(ROI.toArray()), new Point(cx, cy), false) > 0) {
//                Sample sample = new Sample();
//                sample.contour = contour;
//                sample.area = moments.get_m00();
//                sample.centroid = new Point(cx, cy);
//                sample.field_pos = field_pos(sample.centroid);
//
//                tempSampleListRed.add(sample);
//            }
//        }
//    }
//
//    public Mat processFrame(Mat image) {
//        sampleListRed.clear();
//        Mat input = image.clone();
//
//        Mat redMask1 = preprocess(input, lowRed1, highRed1);
//        Mat redMask2 = preprocess(input, lowRed2, highRed2);
//        Mat redMask = new Mat();
//        Core.add(redMask1, redMask2, redMask);
//
//        Mat blueMask = preprocess(input, lowBlue, highBlue);
//        Mat yellowMask = preprocess(input, lowYellow, highYellow);
//
//        List<MatOfPoint> redContours = findContour(redMask);
//        List<MatOfPoint> blueContours = findContour(blueMask);
//        List<MatOfPoint> yellowContours = findContour(yellowMask);
//
//        analyzeContour(redContours);
//        sampleListRed = new ArrayList<>(tempSampleListRed);
//
//        drawContours(input, redContours, "RED");
//
////        Imgproc.polylines(input, ROI.toList(), true, new Scalar(0, 255, 0), 3);
//
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2RGB);
//
//        return input;
//    }
//
//    private Mat preprocess(Mat image, Scalar low, Scalar high) {
//        Mat hsv = new Mat();
//        Imgproc.cvtColor(image, hsv, Imgproc.COLOR_BGR2HSV);
//        Mat mask = new Mat();
//        Core.inRange(hsv, low, high, mask);
//        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
//        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
//
//        return mask;
//    }
//
//    private List<MatOfPoint> findContour(Mat mask) {
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        return contours;
//    }
//
//    private void drawContours(Mat input, List<MatOfPoint> contours, String color) {
//        Scalar objColor;
//
//        switch (color) {
//            case "RED":
//                objColor = new Scalar(0, 0, 255);
//                break;
//            case "BLUE":
//                objColor = new Scalar(255, 0, 0);
//                break;
//            case "YELLOW":
//                objColor = new Scalar(0, 255, 255);
//                break;
//            default:
//                objColor = new Scalar(0, 0, 0);
//                break;
//        }
//
//        for (MatOfPoint contour : contours) {
//            Moments moments = Imgproc.moments(contour);
//            int cX = (int) (moments.get_m10() / moments.get_m00());
//            int cY = (int) (moments.get_m01() / moments.get_m00());
//
//            if (moments.get_m00() > 400 && Imgproc.pointPolygonTest(new MatOfPoint2f(ROI.toArray()), new Point(cX, cY), false) > 0) {
//                Imgproc.drawContours(input, List.of(contour), -1, objColor, 3);
//                Imgproc.circle(input, new Point(cX, cY), 10, new Scalar(0, 255, 0), -1);
//            }
//        }
//    }
//
//    private Point field_pos(Point centroid) {
//        double x = 960 - centroid.x;
//        double y = centroid.y;
//
//        double cross_r = (img_ref1_y - y) * (img_ref2_y - img_ref3_y) * (irl_ref3_y - irl_ref1_y) /
//                ((img_ref1_y - img_ref3_y) * (img_ref2_y - y) * (irl_ref3_y - irl_ref2_y));
//
//        double field_x = (irl_ref2_y * cross_r - irl_ref1_y) / (cross_r - 1);
//        double y_width_x = field_x * (irl_ref2_x - irl_ref1_x) / (irl_ref2_y - irl_ref1_y) + irl_ref1_x;
//        double field_y = x / 960 * y_width_x;
//
//        return new Point(field_x + field_x_offset, field_y + field_y_offset);
//    }
//}
