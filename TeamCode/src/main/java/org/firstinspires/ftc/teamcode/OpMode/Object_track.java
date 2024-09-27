package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lifter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.SampleMarkingPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "Track & Pickup")
public class Object_track extends LinearOpMode {

    private boolean elbowUpLastState = false;
    private boolean elbowDownLastState = false;
    private List<Action> runningActions = new ArrayList<>();
    private final FtcDashboard dash = FtcDashboard.getInstance();
    SampleMarkingPipeline pipeline = new SampleMarkingPipeline();

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 960; // height of wanted camera resolution

    Gamepad C1 = new Gamepad();
    Gamepad P1 = new Gamepad();

    int turret_pos = 0;
    int lifter_pos = Globals.lifterInit;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        Intake intake = new Intake(robot);
        Lifter lifter = new Lifter(robot);
        Turret turret = new Turret(robot);

        intake.updateGripperState(Intake.GripperState.OPEN);
        intake.updateArmState(Intake.ArmState.INIT);
        intake.updateWristState(Intake.WristState.INIT);
        intake.updateElbowState(Intake.ElbowState.INIT);
        lifter.updateLifterState(Lifter.LifterState.INIT);

        List<SampleMarkingPipeline.AnalyzedStone> Samples = Collections.emptyList();
        SampleMarkingPipeline.AnalyzedStone Target = null;
        ElapsedTime timer = new ElapsedTime();
        boolean isFrameCaptured = false;

        initCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            P1.copy(C1);
            C1.copy(gamepad1);

            //Drivetrain
            drive.setDrivePowers(driveCommand(C1));

            //Telemetry
            TelemetryPacket packet = new TelemetryPacket();

            if (C1.left_bumper && !P1.left_bumper) {
                turret_pos+=10;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() + 0.1);
            }
            if (C1.right_bumper && !P1.right_bumper) {
                turret_pos-=10;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() - 0.1);
            }
            if (C1.dpad_left && !P1.dpad_left) {
                turret_pos+=1;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() + 0.005);
            }
            if (C1.dpad_right && !P1.dpad_right) {
                turret_pos-=1;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() - 0.005);
            }
            if (C1.dpad_up && !P1.dpad_up) {
                lifter_pos+=50;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() + 0.005);
            }
            if (C1.dpad_down && !P1.dpad_down) {
                lifter_pos-=50;
//                intake.robot.elbow.setPosition(intake.robot.elbow.getPosition() - 0.005);
            }

            lifter.setLifterPosition(lifter_pos);
            telemetry.addData("Lifter", lifter.LifterPos());
            telemetry.addData("Lifter input",lifter_pos);

            turret.setTurretPosition(turret_pos);
            telemetry.addData("Turret", turret.TurretPos());
            telemetry.addData("Turret input",turret_pos);

            if (C1.a && !P1.a && !isFrameCaptured) {
//                timer.reset();
                if (!pipeline.RedSampleList.isEmpty()) {
                    Samples = new ArrayList<>(pipeline.RedSampleList);
                }
                isFrameCaptured = true;
                telemetry.addLine("Frame captured");
            }
            Target = Best_Sample(Samples);
            String Closest = "Closest: (" + String.format("%.3f",Target.field_pos.x) + ", " + String.format("%.3f",Target.field_pos.y) + ")";
            double[] servo_and_heading = InverseKinematics(Target.field_pos);
            telemetry.addLine(Closest);
            telemetry.addLine("Go to field center and press X to pick");
//            telemetry.addData("Target servo","%.3f",servo_and_heading[0]);
//            telemetry.addData("Target heading","%.3f",servo_and_heading[1]);

            if(C1.x && !P1.x && isFrameCaptured){
                Actions.runBlocking(drive.actionBuilder(drive.pose)
//                        .turn(servo_and_heading[1])
                        .afterTime(1,
                                new SequentialAction(
                                        new InstantAction(()->intake.setElbow(servo_and_heading[0])),
                                        new SleepAction(2)
                                )
                        )
                        .build());
            }

            if (C1.b && !P1.b && isFrameCaptured) {
                isFrameCaptured = false;
                Samples.clear();
                lifter.liftToGround();
                intake.updateElbowState(Intake.ElbowState.DOWN);
            }

            dash.sendTelemetryPacket(packet);
            telemetry.addData("Frame capture",isFrameCaptured);
            telemetry.update();
        }
    }

    private PoseVelocity2d driveCommand(Gamepad gamepad) {
        double drive = -gamepad.left_stick_y * 0.5;
        double strafe = -gamepad.left_stick_x * 0.5;
        double turn = -gamepad.right_stick_x * 0.5;
        return new PoseVelocity2d(new Vector2d(drive, strafe), turn);
    }

    private double[] InverseKinematics(Point obj) {
        double l1 = Globals.elbow_1;
        double l2 = Globals.elbow_2;
        double h = Globals.offset_vertical;
        double d = Math.sqrt(Math.pow(obj.x+Globals.x_offset_center, 2) + Math.pow(obj.y, 2)) - Globals.center_offset_gripper;

        double num = 0;
        double den = 0;
        double cos_term = 90;
        double tan_term = 0;
        if(d<17.5 && d>6){
            num = l1 * l1 + h * h + d * d - l2 * l2;
            den = 2 * l1 * Math.sqrt(d * d + h * h);
            cos_term = Math.acos(num / den)*180/Math.PI;
            tan_term = Math.atan(h / (d))*180/Math.PI;
        }

        double servo_target_pos = Theta2ServoPos(90 + tan_term - cos_term);
        double heading = Math.atan(obj.y / (obj.x + Globals.x_offset_center));

        return new double[]{servo_target_pos,heading};
    }

    private double Theta2ServoPos(double theta) {
        return Globals.elbow0deg + (Globals.elbow55_823deg - Globals.elbow0deg) * theta / 55.832;
    }

    private void initCamera() {
        // Create an instance of the camera
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

    private SampleMarkingPipeline.AnalyzedStone Best_Sample(List<SampleMarkingPipeline.AnalyzedStone> SampleList) {
        SampleMarkingPipeline.AnalyzedStone Best_Sample = new SampleMarkingPipeline.AnalyzedStone();
        Best_Sample.field_pos = new Point(1000,0);
        Best_Sample.area = 0;
        Best_Sample.centroid = new Point(0,0);

        if (!SampleList.isEmpty()) {
            for (SampleMarkingPipeline.AnalyzedStone Sample : SampleList) {
                double dist = Math.sqrt(Math.pow(Sample.field_pos.x + Globals.x_offset_center,2) + Math.pow(Sample.field_pos.y,2));
                double best_dist = Math.sqrt(Math.pow(Best_Sample.field_pos.x + Globals.x_offset_center,2) + Math.pow(Best_Sample.field_pos.y,2));
                if (dist<best_dist){
                    Best_Sample.field_pos = Sample.field_pos;
                }
            }
        }
        return Best_Sample;
    }
}
