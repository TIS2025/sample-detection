package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Camera Test")
@Config
public class ColorCameraTest extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double width = 30;


    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 1.4845;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 462.24;  //

    public static Scalar highRed = new Scalar(130, 255, 255);
    public static Scalar lowRed = new Scalar(120, 70, 50);
    public static Scalar highBlue = new Scalar(30, 255, 255);
    public static Scalar lowBlue = new Scalar(0, 50, 30);//70,50
    public static Scalar highYellow = new Scalar(100, 255, 255);
    public static Scalar lowYellow = new Scalar(90, 70, 50);

    public static int custom_flag = 1;
    public static int r_width = 100;


    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public double c_error = 0.0;
    public double p_error = 0.0;

    public enum MaskColor {
        YELLOW,
        BLUE,
        RED
    }

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor1");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
//            turret.setPower(turretPID(c_error,p_error));

//            timer.reset();
//            p_error = c_error;
            if(gamepad1.dpad_left) turret.setPower(0.5);
            if(gamepad1.dpad_right) turret.setPower(-0.5);

            telemetry.addData("Turret Pos:", turret.getCurrentPosition());
            telemetry.update();
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    class SampleDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect regions
            Mat yellowMask = preprocessFrame(input, lowYellow, highYellow);
            Mat redMask = preprocessFrame(input, lowRed, highRed);
            Mat blueMask = preprocessFrame(input, lowBlue, highBlue);

            // Find contours of the detected regions
            List<MatOfPoint> yellowContours = findContours(yellowMask);
            List<MatOfPoint> redContours = findContours(redMask);
            List<MatOfPoint> blueContours = findContours(blueMask);

            // Display all contours
            drawAllContours(input,yellowContours,MaskColor.YELLOW);
            drawAllContours(input,blueContours,MaskColor.BLUE);
            drawAllContours(input,redContours,MaskColor.RED);

            // Find the largest contours (blob)
            MatOfPoint largestYellowContour = findLargestContour(yellowContours);
            MatOfPoint largestBlueContour = findLargestContour(blueContours);
            MatOfPoint largestRedContour = findLargestContour(redContours);

            gridDetection(input,3,3,yellowContours,redContours,blueContours);


//            if (largestYellowContour != null) drawLargestContour(input, yellowContours, largestYellowContour, MaskColor.YELLOW);
//            if (largestRedContour != null) drawLargestContour(input, redContours, largestRedContour, MaskColor.RED);
//            if (largestBlueContour != null) drawLargestContour(input, blueContours, largestBlueContour, MaskColor.BLUE);
//
//            switch (custom_flag) {
//                case 1:
//                    input = yellowMask;
//                break;
//                case 2:
//                    input = redMask;
//                break;
//                case 3:
//                    input = blueMask;
//                break;
//            }
            return input;
        }

        private Mat preprocessFrame(Mat frame, Scalar low, Scalar high) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat Mask = new Mat();
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

        private void drawLargestContour(Mat input, List<MatOfPoint> contours, MatOfPoint largestContour, MaskColor maskColor) {
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
            width = calculateWidth(largestContour);

//             Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

//            Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);


            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            c_error = Math.atan((cX - CAMERA_WIDTH / 2)*180/getDistance(width)/17.74/Math.PI);

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            String areaLabel = "Area: " + String.valueOf((int) moments.get_m00());
            Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            // Display the object color
            Imgproc.putText(input, obj_color, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            Imgproc.rectangle(input,
                    new Point(CAMERA_WIDTH / 2 - r_width, CAMERA_HEIGHT),
                    new Point(CAMERA_WIDTH / 2 + r_width, 0),
                    new Scalar(255, 255, 255),
                    1,
                    8,
                    0);
        }

        private void drawAllContours(Mat input, List<MatOfPoint> contours, MaskColor maskColor){
            String obj_color = (maskColor == MaskColor.RED) ? "R" :
                    (maskColor == MaskColor.BLUE) ? "B" :
                            (maskColor == MaskColor.YELLOW) ? "Y" : "";

            for (MatOfPoint contour:contours){

                Moments moments = Imgproc.moments(contour);
                int area = (int) moments.get_m00();
                cX = moments.get_m10() / area;
                cY = moments.get_m01() / area;

//                if (area>400 && cY>50 && cY<315) {
                    // Draw a green outline around the contour object
                    Imgproc.drawContours(input, contours,
                            contours.indexOf(contour),
                            new Scalar(0, 255, 0), 2);

                    // Draw a dot at the centroid
                    String label = "(" + (int) cX + ", " + (int) cY + ")";
                    Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                    Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                    // Calculate area and write it
                    String areaLabel = "Area: " + String.valueOf((int) moments.get_m00());
                    Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                    // Display the contour color
                    Imgproc.putText(input, obj_color, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 2);
//                }
            }
        }

        private void gridDetection(Mat input, int rows, int columns,List<MatOfPoint> yellowContours,List<MatOfPoint> redContours, List<MatOfPoint> blueContours) {
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

                        if(gridCell.contains(new Point(x,y))) blueArea += moments.get_m00();
                    }

                    for (MatOfPoint contour : yellowContours) {
                        Moments moments = Imgproc.moments(contour);
                        double x = moments.get_m10() / moments.get_m00();
                        double y = moments.get_m01() / moments.get_m00();

                        if(gridCell.contains(new Point(x,y))) yellowArea += moments.get_m00();
                    }

                    for (MatOfPoint contour : redContours) {
                        Moments moments = Imgproc.moments(contour);
                        double x = moments.get_m10() / moments.get_m00();
                        double y = moments.get_m01() / moments.get_m00();

                        if(gridCell.contains(new Point(x,y))) redArea += moments.get_m00();
                    }

                    String dominantColor = "None";
                    double maxArea = Math.max(blueArea, Math.max(yellowArea, redArea));

                    if (maxArea == blueArea) dominantColor = "Blue";
                    else if (maxArea == yellowArea) dominantColor = "Yellow";
                    else if (maxArea == redArea) dominantColor = "Red";

                    if(maxArea <= 1000) dominantColor = "None";

                    Scalar color = null;

                    switch (dominantColor) {
                        case "Blue":
                            color = new Scalar(0, 0, 255); // Blue color

                            break;
                        case "Yellow":
                            color = new Scalar(0, 255, 255); // Yellow color

                            break;
                        case "Red":
                            color = new Scalar(255, 0, 0); // Red color

                            break;
                        case "None":
                            color = new Scalar(255, 255, 255); // Default color (White for no dominant color)
                            break;
                    }

                    // Draw rectangle around the grid cell with the dominant color
                    Imgproc.rectangle(input, new Point(xStart+2, yStart+2), new Point(xEnd-2, yEnd-2), color, 2);
                    Imgproc.putText(input, "Max:"+maxArea, new Point((double) (xStart + xEnd) /2, (double) (yStart + yEnd) /2), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(255, 255, 255), 2);

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

    class EdgeDetectionPipeline extends OpenCvPipeline{

        @Override
        public Mat processFrame(Mat input) {
            Mat gray = preProcess(input);

            Mat edges = detectEdges(gray);

            List<MatOfPoint> lines = detectLines(edges);

            return drawLines(input, lines);
        }

        private Mat preProcess(Mat src) {
            Mat gray = new Mat();
            // Convert the image to grayscale
            Imgproc.cvtColor(src, gray, Imgproc.COLOR_BGR2GRAY);
            // Apply Gaussian blur to smooth the image
            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
            return gray;
        }

        private Mat detectEdges(Mat gray) {
            Mat edges = new Mat();
            // Apply Canny Edge Detection
            Imgproc.Canny(gray, edges, 50, 150);
            return edges;
        }

        private List<MatOfPoint> detectLines(Mat edges) {
            Mat lines = new Mat();
            List<MatOfPoint> linePoints = new ArrayList<>();
            // Detect lines using Hough Line Transform
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 100, 50, 10);

            for (int i = 0; i < lines.rows(); i++) {
                double[] line = lines.get(i, 0);
                Point pt1 = new Point(line[0], line[1]);
                Point pt2 = new Point(line[2], line[3]);

                double angle = Math.atan2(pt2.y - pt1.y, pt2.x - pt1.x);

                if (Math.abs(angle) < Math.PI / 18) { // pi/18 ~ 10 degrees tolerance for horizontal lines
                    linePoints.add(new MatOfPoint(pt1, pt2));
                }
            }
            return linePoints;
        }

        private Mat drawLines(Mat src, List<MatOfPoint> lines) {
            for (MatOfPoint line : lines) {
                Point[] points = line.toArray();
                // Draw each line segment on the source image
                Imgproc.line(src, points[0], points[1], new Scalar(0, 255, 0), 3);
            }
            return src;
        }
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//        controlHubCam.setPipeline(new SampleDetectionPipeline());
        controlHubCam.setPipeline(new SampleDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    private double turretPID(double cError, double pError) {
        return Kp*c_error + Kd*(cError-pError)/(timer.seconds());
    }

    private static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}
