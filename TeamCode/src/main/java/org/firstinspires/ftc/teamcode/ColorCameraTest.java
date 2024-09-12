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

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
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

            telemetry.addData("Angular error:", c_error);
            telemetry.update();

//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private double turretPID(double cError, double pError) {
        return Kp*c_error + Kd*(cError-pError)/(timer.seconds());
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

            // Find the largest yellow contour (blob)
            MatOfPoint largestYellowContour = findLargestContour(yellowContours);
            MatOfPoint largestBlueContour = findLargestContour(blueContours);
            MatOfPoint largestRedContour = findLargestContour(redContours);

            if (largestYellowContour != null) {
                drawContour(input, yellowContours, largestYellowContour, MaskColor.YELLOW);
            }
            if (largestRedContour != null) {
                drawContour(input, redContours, largestRedContour, MaskColor.RED);
            }
            if (largestBlueContour != null) {
                drawContour(input, blueContours, largestBlueContour, MaskColor.BLUE);
            }
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

        private void drawContour(Mat input, List<MatOfPoint> contours, MatOfPoint largestContour, MaskColor maskColor) {
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

            Imgproc.drawContours(input, contours, contours.indexOf(largestContour)
                    , new Scalar(0, 255, 0), 2);

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

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }
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

    private static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}
