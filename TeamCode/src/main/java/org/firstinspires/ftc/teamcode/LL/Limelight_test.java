package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.VisionUtils.CameraOrientation;
import org.firstinspires.ftc.teamcode.VisionUtils.KinematicSolver;
import org.firstinspires.ftc.teamcode.VisionUtils.PerspectiveSolver;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Limelight Matrix")
public class Limelight_test extends LinearOpMode {
    public static Limelight3A limelight;
    public DcMotorEx ext;

    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;

    public List<Sample> redSamples = new ArrayList<>();
    public List<Sample> blueSamples = new ArrayList<>();
    public List<Sample> yellowSamples = new ArrayList<>();

    double w1 = 6.339;
    double w2 = 16.023;
    double cx1 = CAMERA_HEIGHT - 480;
    double cx2 = CAMERA_HEIGHT - 234;
    double cx3 = CAMERA_HEIGHT - 178;
    public static double angle = 0;
    double cam_offset = 5.75;
    double x_offset = 0;
    double y_offset = 0;
    Point obj_pos = new Point(0,0);
    Point[] prev_pos = {new Point(0,0),new Point(0,0),new Point(0,0)};
    boolean obj_orient = false;

    PerspectiveSolver Psolver = new PerspectiveSolver(angle,x_offset,y_offset,cam_offset,cx1,cx2,cx3,0,10,20,
            0,0,0,0,0,0,w1,w2, CameraOrientation.UPRIGHT,CAMERA_HEIGHT,CAMERA_WIDTH);

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);
        ext = hardwareMap.get(DcMotorEx.class,"xExtension");
//        ext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();

        limelight.pipelineSwitch(0);

        waitForStart();
        limelight.start();
        while (opModeIsActive()) {
            P.copy(C);
            C.copy(gamepad1);
            redSamples.clear();
            blueSamples.clear();
            yellowSamples.clear();


            LLResult result = limelight.getLatestResult();

            sample_filter(result);

            if(!redSamples.isEmpty()) {
                telemetry.addData("Red Samples",redSamples.size());
                telemetry.addData("Sample",redSamples.get(0).field_pos);
                telemetry.addData("Orientation",redSamples.get(0).orientation);
            }
            else telemetry.addData("Red Samples",0);

            if(!blueSamples.isEmpty())  {
                telemetry.addData("Blue Samples",blueSamples.size());
                telemetry.addData("Sample",blueSamples.get(0).field_pos);
                telemetry.addData("Orientation",blueSamples.get(0).orientation);
            }
            else telemetry.addData("Blue Samples",0);

            if(!yellowSamples.isEmpty())  {
                telemetry.addData("Yellow Samples",yellowSamples.size());
                telemetry.addData("Sample",yellowSamples.get(0).field_pos);
                telemetry.addData("Orientation",yellowSamples.get(0).orientation);
            }
            else telemetry.addData("Yellow Samples",0);

            if(C.y && !P.y && !redSamples.isEmpty()){
                ExtInchInput(redSamples.get(0).field_pos.x - 6);
                obj_orient = redSamples.get(0).orientation;
            }

            if(C.left_bumper && !P.left_bumper){
                ExtTickInput(300);
            }
            if(C.right_bumper && !P.right_bumper){
                ExtTickInput(600);
            }

            if(gamepad1.x){
                ExtTickInput(0);
            }


            telemetry.addData("Ext Pos",ext.getCurrentPosition());
            telemetry.update();
        }

        limelight.stop();

    }

    public void ExtInchInput(double target){
        int final_target = (int)(target * 1035.0/15);

        final_target +=ext.getCurrentPosition();
        final_target = Math.max(Math.min(final_target,1050),0);
        ext.setTargetPosition(final_target);
        ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ext.setPower(1);
    }

    public void ExtTickInput(int target){
        target = Math.max(Math.min(target,1050),0);
        ext.setTargetPosition(target);
        ext.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ext.setPower(1);
    }

    public void interpret_limelight(LLResult result){

        try{
            telemetry.addData("No of samples detected",result.getDetectorResults().size());
            List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
            List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
            List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
            List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);

            double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
            double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
            double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
            double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

            double width = max_x-min_x;
            double height = max_y-min_y;

            double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
            double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

            Point centroid = new Point(cx, cy);
            obj_pos = Psolver.getX2Y2(centroid);
            obj_orient = width/height>1.2;
            telemetry.addData("Width/Height","%.3f",width/height);
            double offset = obj_orient? 0.5:1.5;
            obj_pos.x+=offset;
            prev_pos[0] = prev_pos[1];
            prev_pos[1] = prev_pos[2];
            prev_pos[2] = obj_pos;
        }
        catch (Exception e){
            obj_pos = new Point(0,0);
        }
    }

    public void sample_filter(LLResult result){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                List<Double> pt1 = detector.getTargetCorners().get(0);
                List<Double> pt2 = detector.getTargetCorners().get(1);
                List<Double> pt3 = detector.getTargetCorners().get(2);
                List<Double> pt4 = detector.getTargetCorners().get(3);

                double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

                double width = max_x - min_x;
                double height = max_y - min_y;

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = Psolver.getX2Y2(new Point(cx, cy));
                boolean orientation = width/height>1.2;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();
                double offset = orientation?0.5:1.5;

                if (Math.abs(field_pos.y) < 6 && field_pos.x < 15 && class_id == 0) {
                    field_pos.x+=offset;
                    blueSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y) < 6 && field_pos.x < 15 && class_id == 1) {
                    field_pos.x+=offset;
                    redSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y) < 6 && field_pos.x < 15 && class_id == 2) {
                    field_pos.x+=offset;
                    yellowSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
            }
        }
        catch (Exception e){
            telemetry.addLine("Failed to input samples");
        }
    }


}
