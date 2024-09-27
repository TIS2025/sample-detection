package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
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

    //Telemetry

    //Submersible
//    Point top_left = new Point(222, 141);
//    Point top_right = new Point(418, 141);
//    Point bottom_left = new Point(0, 335);
//    Point bottom_right = new Point(640, 335);
//    Point mid_left = new Point(168,188);
//    Point mid_right = new Point(472,188);
    Point top_left = new Point(444, 282);
    Point top_right = new Point(836, 282);
    Point bottom_left = new Point(0, 670);
    Point bottom_right = new Point(1280, 670);
    Point mid_left = new Point(336,376);
    Point mid_right = new Point(944,376);

    Point[] points = new Point[]{
            top_left,
            top_right,
            mid_right,
            bottom_right,
            bottom_left,
            mid_left
    };

    MatOfPoint submersible_contour = new MatOfPoint(points);

    //Camera parameters
    public static final double fx = 463.94,fy = 462.24;
    public static final double camX = 0,camY = 0;

//    int CAMERA_HEIGHT = 480;
//    int CAMERA_WIDTH = 640;

    int CAMERA_HEIGHT = 960;
    int CAMERA_WIDTH = 1280;

    public enum MaskColor {
        YELLOW,
        BLUE,
        RED
    }

    Mat grid = Mat.zeros(10, 10, CvType.CV_8UC1);

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    double cX,cY;

    public static class AnalyzedStone{
        MatOfPoint contour;
        public double area;
        public Point centroid;
        public Point field_pos;
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


    public SampleMarkingPipeline(){
        cameraMatrix.put(0, 0,
                  fx, 0, camX,
                        0, fy, camY,
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

        Imgproc.line(input,top_right,top_left, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,top_right,mid_right, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,mid_right,bottom_right, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,bottom_right,bottom_left, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,bottom_left,mid_left, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,mid_left,top_left, new Scalar(0, 255, 0), 2);
        Imgproc.line(input,mid_left,mid_right, new Scalar(0, 255, 0), 2);

        // Find the largest contours (blob)
//        MatOfPoint largestYellowContour = findLargestContour(yellowContours);
//        MatOfPoint largestBlueContour = findLargestContour(blueContours);
//        MatOfPoint largestRedContour = findLargestContour(redContours);

//        gridDetection(input, 10, 10, yellowContours, redContours, blueContours);


//        target = targetGridElement(grid, SampleDetectionPipeline.MaskColor.RED);

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

    private void AnalyzeContour(List<MatOfPoint> contours,ArrayList<SampleMarkingPipeline.AnalyzedStone> samplelist){
        samplelist.clear();
        for (MatOfPoint contour: contours){
            AnalyzedStone Sample = new AnalyzedStone();
            Moments moments = Imgproc.moments(contour);
            int area = (int) moments.get_m00();

            if (area>100){
//                Sample.contour = contour;
                Point cent = new Point(moments.get_m10() / area,moments.get_m01() / area);

                if(Imgproc.pointPolygonTest(new MatOfPoint2f(submersible_contour.toArray()),cent,false)>0){
                    Sample.contour = contour;
                    Sample.centroid = cent;
                    Sample.area = area;
                    Sample.field_pos = locate_point(Sample.centroid);

                    samplelist.add(Sample);
                }
            }
        }
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

    private void drawLargestContour(Mat input, List<MatOfPoint> contours, MatOfPoint largestContour, SampleDetectionPipeline.MaskColor maskColor) {
        // Draw a green outline around the largest detected object
        String obj_color = "";

        switch (maskColor) {
            case RED:
                obj_color = "Red";
                break;
            case BLUE:
                obj_color = "Blue";
                break;
            case YELLOW:
                obj_color = "Yellow";
                break;
        }

        Imgproc.drawContours(input, contours,
                contours.indexOf(largestContour),
                new Scalar(0, 255, 0), 2);

        // Calculate the width of the bounding box
//        width = calculateWidth(largestContour);
//
////             Display the width next to the label
//        String widthLabel = "Width: " + (int) width + " pixels";
//        Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

//            Display the Distance
//        String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
//        Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);


        // Calculate the centroid of the largest contour
        Moments moments = Imgproc.moments(largestContour);
        cX = moments.get_m10() / moments.get_m00();
        cY = moments.get_m01() / moments.get_m00();

        // Draw a dot at the centroid
        String label = "(" + (int) cX + ", " + (int) cY + ")";
        Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        String areaLabel = "Area: " + String.valueOf((int) moments.get_m00());
        Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

        // Display the object color
        Imgproc.putText(input, obj_color, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

//        Imgproc.rectangle(input,
//                new Point(CAMERA_WIDTH / 2 - r_width, CAMERA_HEIGHT),
//                new Point(CAMERA_WIDTH / 2 + r_width, 0),
//                new Scalar(255, 255, 255),
//                1,
//                8,
//                0);
    }

    private void drawAllContours(Mat input, ArrayList<SampleMarkingPipeline.AnalyzedStone> samplelist, SampleDetectionPipeline.MaskColor maskColor) {
        String obj_color = (maskColor == SampleDetectionPipeline.MaskColor.RED) ? "R" :
                (maskColor == SampleDetectionPipeline.MaskColor.BLUE) ? "B" :
                        (maskColor == SampleDetectionPipeline.MaskColor.YELLOW) ? "Y" : "";

        for (AnalyzedStone sample : samplelist) {

            int area = (int) sample.area;
            cX = sample.centroid.x;
            cY = sample.centroid.y;

//                if (area>400 && cY>50 && cY<315) {
            // Draw a green outline around the contour object
//            Imgproc.drawContours(input, contours,
//                    contours.indexOf(contour),
//                    new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, sample.centroid, 3, new Scalar(0, 255, 0), -1);

                // Draw a dot at the centroid
                String label = "(" + String.format("%.2f",sample.field_pos.x) + ", " + String.format("%.2f",sample.field_pos.y) + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 1);

                // Calculate area and write it
//            String areaLabel = "Area: " + String.valueOf((int) moments.get_m00());
//            Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                // Display the contour color
//                Imgproc.putText(input, obj_color, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 2);
        }
    }

    private void gridDetection(Mat input, int rows, int columns, List<MatOfPoint> yellowContours, List<MatOfPoint> redContours, List<MatOfPoint> blueContours) {
        int imageHeight = input.rows();
        int imageWidth = input.cols();

        int cellHeight = imageHeight / rows;
        int cellWidth = imageWidth / columns;

        // Iterating over the grid
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                // Define the ROI (Region of Interest) for the current cell
                int xStart = j * cellWidth;
                int yStart = i * cellHeight;
                int xEnd = Math.min(xStart + cellWidth, imageWidth);
                int yEnd = Math.min(yStart + cellHeight, imageHeight);

                Rect gridCell = new Rect(xStart, yStart, xEnd - xStart, yEnd - yStart);

                double blueArea = 0;
                double yellowArea = 0;
                double redArea = 0;

                for (MatOfPoint contour : blueContours) {
                    Moments moments = Imgproc.moments(contour);
                    double x = moments.get_m10() / moments.get_m00();
                    double y = moments.get_m01() / moments.get_m00();

                    if (gridCell.contains(new Point(x, y))) blueArea += moments.get_m00();
                }

                for (MatOfPoint contour : yellowContours) {
                    Moments moments = Imgproc.moments(contour);
                    double x = moments.get_m10() / moments.get_m00();
                    double y = moments.get_m01() / moments.get_m00();

                    if (gridCell.contains(new Point(x, y))) yellowArea += moments.get_m00();
                }

                for (MatOfPoint contour : redContours) {
                    Moments moments = Imgproc.moments(contour);
                    double x = moments.get_m10() / moments.get_m00();
                    double y = moments.get_m01() / moments.get_m00();

                    if (gridCell.contains(new Point(x, y))) redArea += moments.get_m00();
                }

                String dominantColor = "None";
                double maxArea = Math.max(blueArea, Math.max(yellowArea, redArea));

                if (maxArea == blueArea) dominantColor = "Blue";
                else if (maxArea == yellowArea) dominantColor = "Yellow";
                else if (maxArea == redArea) dominantColor = "Red";

                if (maxArea <= 1000) dominantColor = "None";

                Scalar color = null;
                int col_code = 999;

                switch (dominantColor) {
                    case "Blue":
                        color = new Scalar(0, 0, 255);
                        col_code = 1;// Blue color

                        break;
                    case "Yellow":
                        color = new Scalar(0, 255, 255);
                        col_code = 0;// Yellow color

                        break;
                    case "Red":
                        color = new Scalar(255, 0, 0);
                        col_code = -1;// Red color

                        break;
                    case "None":
                        color = new Scalar(255, 255, 255); // Default color (White for no dominant color)
                        break;
                }

                // Draw rectangle around the grid cell with the dominant color
                grid.put(i, j, col_code);
                Imgproc.rectangle(input, new Point(xStart + 2, yStart + 2), new Point(xEnd - 2, yEnd - 2), color, 2);
//                    Imgproc.putText(input, "Max:"+maxArea, new Point((double) (xStart + xEnd) /2, (double) (yStart + yEnd) /2), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            }
        }

//            // Vertical lines for the grid
//            for (int x = 0; x <= imageWidth; x += cellWidth) {
//                Imgproc.line(input, new Point(x, 0), new Point(x, imageHeight), new Scalar(255, 255, 255), 1);
//            }
//
//            // Horizontal lines for the grid
//            for (int y = 0; y <= imageHeight; y += cellHeight) {
//                Imgproc.line(input, new Point(0, y), new Point(imageWidth, y), new Scalar(255, 255, 255), 1);
//            }

    }

    private Point locate_point(Point centroid){

        double max_per_shift = (top_right.x - (double) CAMERA_WIDTH /2) / (bottom_right.x - (double) CAMERA_WIDTH /2);
        double field_height = bottom_left.y - top_left.y;
        double field_width = bottom_right.x - bottom_left.x;
        double tile1_height = bottom_right.y - mid_right.y;
        double tile2_height = mid_right.y - top_right.y;

        double d_const_x = 44/field_height;
        double d_const_y = 26/field_width;

        double x = bottom_right.y - centroid.y;
        double y = (((double) CAMERA_WIDTH /2) - centroid.x);

        double cross_ratio = 44.0/22.0*(tile1_height)/(field_height)*(field_height-x)/(tile1_height-x);

        double const_lat = 1 - (1 - max_per_shift) * x / field_height;

        double field_x = (22*cross_ratio - 44)/(cross_ratio - 1);
        //Correction for center of object vs base of object
        field_x -= (1 * x/field_height)/d_const_x;

        double field_y = y/const_lat * d_const_y;

        return new Point(field_x,field_y);
    }

    private int[] targetGridElement(Mat grid, SampleDetectionPipeline.MaskColor color) {
        int target = 999;
        switch (color) {
            case RED:
                target = -1;
            case BLUE:
                target = 1;
            case YELLOW:
                target = 0;
        }

        int cols = grid.cols();
        int rows = grid.rows();
        int min_distance = 999;

        int[] target_grid = {rows - 1, (int) (cols / 2)};

        for (int i = rows - 1; i >= 2; i--) {
            for (int j = 0; j < cols; j++) {
                if ((int) (grid.get(i, j)[0]) == target) {
                    int distance = Math.abs(cols / 2 - j);
                    if (distance < min_distance) {
                        target_grid[0] = i;
                        target_grid[1] = j;
                        min_distance = distance;
                    }
                }
            }
            if (min_distance < 999) break;
        }
        return target_grid;
    }

    public double contourAreaInCell(MatOfPoint contour, Rect cellROI) {
        // Create a mask for the cell ROI
        Mat mask = Mat.zeros(cellROI.size(), CvType.CV_8UC1);

        // Offset the contour points to the current ROI
        MatOfPoint contourOffset = new MatOfPoint();
        for (Point point : contour.toArray()) {
            if (cellROI.contains(point)) {
                contourOffset.push_back(new MatOfPoint(new Point(point.x - cellROI.x, point.y - cellROI.y)));
            }
        }

        // Calculate the contour area within the mask
        return Core.countNonZero(mask);
    }

    public boolean contourOverlapsWithCell(MatOfPoint contour, Rect cellROI) {
        for (Point point : contour.toArray()) {
            if (cellROI.contains(point)) {
                return true;
            }
        }
        return false;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

}
