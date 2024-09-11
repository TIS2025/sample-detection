package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
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
import java.util.List;

@TeleOp(name = "Camera Test")
@Config
public class ColorCameraTest extends LinearOpMode{
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  //

    public static Scalar highRed = new Scalar(130,255,255);
    public static Scalar lowRed = new Scalar(120,70,50);
    public static Scalar highBlue = new Scalar(30,255,255);
    public static Scalar lowBlue = new Scalar(0,70,50);
    public static Scalar highYellow = new Scalar(100,255,255);
    public static Scalar lowYellow = new Scalar(90,70,50);
//    public static int lowHue = 100;
//    public static int highSat = 255;
//    public static int lowSat = 100;
//    public static int highVal = 255;
//    public static int lowVal = 100;

//    Red - 120-130, Blue - 0-30, Yellow - 90-100
//    Sat - 70-255, Val - 50-255

    @Override
    public void runOpMode() throws InterruptedException {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new SampleDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    private static double getDistance(double width){
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }

    class SampleDetectionPipeline extends OpenCvPipeline {

        public enum MaskColor{
            YELLOW,
            BLUE,
            RED
        }

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect regions
            Mat yellowMask = preprocessFrame(input,lowYellow,highYellow);
            Mat redMask = preprocessFrame(input,lowRed,highRed);
            Mat blueMask = preprocessFrame(input,lowBlue,highBlue);

            // Find contours of the detected regions
            List<MatOfPoint> yellowContours = findContours(yellowMask);
            List<MatOfPoint> redContours = findContours(redMask);
            List<MatOfPoint> blueContours = findContours(blueMask);

            // Find the largest yellow contour (blob)
            MatOfPoint largestYellowContour = findLargestContour(yellowContours);
            MatOfPoint largestBlueContour = findLargestContour(blueContours);
            MatOfPoint largestRedContour = findLargestContour(redContours);

            if (largestYellowContour != null) {
            }
            return input;
        }

        private Mat preprocessFrame(Mat frame, Scalar low, Scalar high) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat Mask = new Mat();
            Core.inRange(hsvFrame, lowRed, highRed, Mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_CLOSE, kernel);

            return Mask;
        }

        private List<MatOfPoint> findContours(Mat mask){
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

        private void drawContour(Mat input,List<MatOfPoint> contours, MatOfPoint largestContour){
            // Draw a green outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour)
                    , new Scalar(0, 255, 0), 2);

            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";

            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

//                double[] hsvPixel = yellowMask.get((int)cX,(int)cY);
//                String hueLabel = "Hue: " + String.format(String.valueOf(hsvPixel[0]));
//                Imgproc.putText(input, hueLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
}
