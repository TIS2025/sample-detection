package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RedBlueDetectionPipelineNoPNP;
import org.firstinspires.ftc.teamcode.vision.SampleMarkingPipeline;
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
import java.util.List;

@TeleOp(name = "Camera Test")
@Config
public class ColorCameraTest extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double width = 30;

    double camera_x;
    double camera_y;

    public static double x_con = 0,y_con = 0;

    int grid_rows=8,grid_cols=10;

    SampleMarkingPipeline pipeline = new SampleMarkingPipeline();

    int[] target = {0,0};
    Mat grid = Mat.zeros(grid_rows,grid_cols,CvType.CV_8UC1);

    double gripper_x_offset = 6;
    double gripper_y_offset = 7;



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

        double fx = 0;
        double fy = 0;


        waitForStart();

        while (opModeIsActive()) {

            if(!pipeline.RedSampleList.isEmpty()){
                telemetry.addData("Red size",pipeline.RedSampleList.size());
                fx = pipeline.RedSampleList.get(0).field_pos.x;
                fy = pipeline.RedSampleList.get(0).field_pos.y;
            }

            telemetry.addData("Field x",fx);
            telemetry.addData("Field y",fy);

//            telemetry.addData("Blue size",pipeline.BlueSampleList.size());
            telemetry.update();
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

//        controlHubCam.setPipeline(new SampleDetectionPipeline());
        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    private static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}
