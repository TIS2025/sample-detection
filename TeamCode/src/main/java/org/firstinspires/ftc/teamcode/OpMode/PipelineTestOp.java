package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ObjectTrackPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionConst;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "Object track and pick",group = "Pipeline Testing")
public class PipelineTestOp extends LinearOpMode {

    private OpenCvCamera webcam;
    ObjectTrackPipeline pipeline = new ObjectTrackPipeline();

    @Override
    public void runOpMode() throws InterruptedException {

        // OpenCV webcam
        initCamera();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug);
            }
            List<ObjectTrackPipeline.AnalyzedStone> SampleList = pipeline.getSampleList();
        }

    }

    public void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener()
                {
                    @Override
                    public void onOpened()
                    {
                        webcam.startStreaming(VisionConst.CAM_WIDTH, VisionConst.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    }
                    @Override
                    public void onError(int errorCode) {}
                }
        );
    }
}
