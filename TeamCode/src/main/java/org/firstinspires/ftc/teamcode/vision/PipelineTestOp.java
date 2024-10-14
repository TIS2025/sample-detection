package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Pipeline Tester")
public class PipelineTestOp extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    GripperRotationPipeline pipeline = new GripperRotationPipeline();
    public static int CAMERA_WIDTH = 1280;
    public static int CAMERA_HEIGHT = 960;
    double servo0 = 0.305;
    double servo180 = 0.965;


    @Override
    public void runOpMode() throws InterruptedException {

//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Servo gripper = hardwareMap.get(Servo.class,"grip");
        gripper.setPosition((servo0+servo180)/2);


        initCamera();

        waitForStart();
        while(opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);

            if(C1.dpad_up && !P1.dpad_up) gripper.setPosition(gripper.getPosition()+0.005);
            if(C1.dpad_down && !P1.dpad_down) gripper.setPosition(gripper.getPosition()-0.005);
            if(C1.a && !P1.a) gripper.setPosition(gripper.getPosition()+0.1);
            if(C1.b && !P1.b) gripper.setPosition(gripper.getPosition()-0.1);

            telemetry.addData("Angle",pipeline.getAngle());

            gripper.setPosition(get_servo_pos(pipeline.getAngle()));
            telemetry.addData("Gripper or",gripper.getPosition());
            telemetry.update();
//            drive.setDrivePowers(driveCommand(C1));
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
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
    }

    private double get_servo_pos(double angle){
        return (angle-90)/180*(servo180-servo0) + (servo0+servo180)/2;
    }

    private PoseVelocity2d driveCommand(Gamepad gamepad) {
        double drive = -gamepad.left_stick_y * 0.5;
        double strafe = -gamepad.left_stick_x * 0.5;
        double turn = -gamepad.right_stick_x * 0.5;
        return new PoseVelocity2d(new Vector2d(drive, strafe), turn);
    }
}
