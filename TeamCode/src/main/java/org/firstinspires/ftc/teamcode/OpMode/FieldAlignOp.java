package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals.VisionConst;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.FieldAlignPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Field Align Test")
@Config
public class FieldAlignOp extends LinearOpMode {

    public static double X = VisionConst.startPose.position.x;
    public static double Y = VisionConst.startPose.position.y;
    public static double heading = 90;

    FieldAlignPipeline pipeline = new FieldAlignPipeline();
    public int CAMERA_HEIGHT = 720;
    public int CAMERA_WIDTH = 1280;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap,VisionConst.startPose);
        initCamera();

        waitForStart();
        while (opModeIsActive()){

            drive.updatePoseEstimate();
            double x = drive.pose.position.x;
            double y = drive.pose.position.y;
            double heading = Math.toDegrees(drive.pose.heading.toDouble());
            pipeline.updatePose(x,y,heading);
            telemetry.addData("X",pipeline.X);
            telemetry.addData("Y",pipeline.Y);
            telemetry.addData("heading",pipeline.yaw);
            telemetry.addData("bot x", x);
            telemetry.addData("bot y", y);
            telemetry.addData("bot heading (deg)", heading);
            telemetry.update();
        }
    }

    private void initCamera(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(pipeline);
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);
    }
}
