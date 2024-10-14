package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "field view test")
@Config
public class FieldAlignOp extends LinearOpMode {

    int CAMERA_HEIGHT = 720;
    int CAMERA_WIDTH = 1280;
    FieldAlignPipeline pipeline = new FieldAlignPipeline();
    //    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

//        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
//        drivetrain.setPoseEstimate(new Pose2d(56,0));


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(pipeline);
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        Gamepad CurGP1 = new Gamepad();
        Gamepad PreGP1 = new Gamepad();

        waitForStart();
        while(opModeIsActive()){
            PreGP1.copy(CurGP1);
            CurGP1.copy(gamepad1);

//            drive(drivetrain, CurGP1.left_stick_x * 1.1, -CurGP1.left_stick_y, -CurGP1.right_stick_x);
//            drivetrain.update();
//            Pose2d poseEstimate = drivetrain.getPoseEstimate();
//            pipeline.updateXY(poseEstimate.getY(),poseEstimate.getX());

            telemetry.addData("x", pipeline.X);
            telemetry.addData("y", pipeline.Y);
//            telemetry.addLine("                                      ");
//            telemetry.addData("LeftFrontCurrent", drivetrain.getMotorCurrent().get(0));
//            telemetry.addData("RightFrontCurrent", drivetrain.getMotorCurrent().get(1));
//            telemetry.addData("LeftRearCurrent", drivetrain.getMotorCurrent().get(2));
//            telemetry.addData("RightRearCurrent", drivetrain.getMotorCurrent().get(3));
//            telemetry.addData("heading", poseEstimate.getHeading());
            Point[] field = pipeline.get_field(pipeline.X, pipeline.Y);
            telemetry.addData("p1",field[0]);
            telemetry.addData("p2",field[1]);
            telemetry.addData("p3",field[2]);
            telemetry.addData("p4",field[3]);
            telemetry.update();

        }

    }

    private void initCamera(){


    }
//    public void drive(@NonNull SampleMecanumDrive drivetrain, double x, double y, double rx) {
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//        drivetrain.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
//    }
}
