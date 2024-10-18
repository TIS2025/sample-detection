package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals.VisionConst;
import org.firstinspires.ftc.teamcode.vision.FieldAlignPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Field Align Test")
@Config
public class FieldAlignOp extends LinearOpMode {

    public static double X = 0;
    public static double Y = 0;
    public static double heading = 0;

    FieldAlignPipeline pipeline = new FieldAlignPipeline();
    public int CAMERA_HEIGHT = 720;
    public int CAMERA_WIDTH = 1280;

    @Override
    public void runOpMode() throws InterruptedException {

        initCamera();

        waitForStart();
        while (opModeIsActive()){
            pipeline.updatePose(VisionConst.CAMERA_X,VisionConst.CAMERA_Y,VisionConst.CAMERA_Z,VisionConst.CAMERA_YAW,VisionConst.CAMERA_PITCH,VisionConst.CAMERA_ROLL);
            telemetry.addData("X",pipeline.X);
            telemetry.addData("Y",pipeline.Y);
            telemetry.addData("Z",pipeline.Z);
            telemetry.addData("Pitch",pipeline.pitch);
            telemetry.addData("Yaw",pipeline.yaw);
            telemetry.addData("Roll",pipeline.roll);
            telemetry.addData("top_left",pipeline.tl);
            telemetry.addData("top_right",pipeline.tr);
            telemetry.addData("bottom_left",pipeline.bl);
            telemetry.addData("bottom_right",pipeline.br);
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
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);
    }
}
