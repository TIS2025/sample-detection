package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VisionUtils.CameraOrientation;
import org.firstinspires.ftc.teamcode.VisionUtils.KinematicSolver;
import org.firstinspires.ftc.teamcode.VisionUtils.PerspectiveSolver;
import org.opencv.core.Point;

import java.util.List;


@TeleOp(name = "Limelight pos test")
public class Limelight_test extends LinearOpMode {
    public static Limelight3A limelight;

    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;

    double w1 = 8.25;
    double w2 = 18.25;
    double cx1 = CAMERA_HEIGHT - 480;
    double cx2 = CAMERA_HEIGHT - 266;
    double cx3 = CAMERA_HEIGHT - 207;
    public static double angle = 0;
    double cam_offset = 7;
    double x_offset = 0;
    double y_offset = 0;
    Point obj_pos = new Point(0,0);
    Point[] prev_pos = {new Point(0,0),new Point(0,0),new Point(0,0)};
    boolean obj_orient = false;

    PerspectiveSolver Psolver = new PerspectiveSolver(angle,x_offset,y_offset,cx1,cx2,cx3,0,10,20,
            0,0,0,0,0,0,w1,w2, CameraOrientation.UPRIGHT,CAMERA_HEIGHT,CAMERA_WIDTH);

    KinematicSolver solver = new KinematicSolver(1.5,5.2,0.5,0);
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);

//        limelight.reloadPipeline();
        limelight.pipelineSwitch(2);

////        limelight.stop();
//        while(opModeInInit()){
//            if(!limelight.isConnected()){
////                limelight.setPollRateHz(100);
//                limelight.stop();
//                sleep(1000);
//                limelight.start();
//            }

//
//            telemetry.addData("Connected",limelight.isConnected());
//            telemetry.addData("Connected",limelight.isRunning());
//            telemetry.update();
//        }
//        limelight.start();
//        Thread.sleep(100);
        waitForStart();
        limelight.start();
        while (opModeIsActive()) {

//            sleep(20);
//            LLResult result = limelight.getLatestResult();
            LLResult result = limelight.getLatestResult();

            try{
                interpret_limelight(result);
//                telemetry.addData("result", result.getDetectorResults().get(0).getTargetCorners());


//                List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
//                List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
//                List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
//                List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);
//
//                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
//                double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;
//
//                Point centroid = new Point(cx, cy);

//                telemetry.addData("centroid", new Point(cx, cy));
                telemetry.addData("obj_pos",obj_pos);
//                telemetry.addData("Field_Pos", get_field_pos(centroid));
            }
            catch (Exception e){
                telemetry.addLine("No point");
            }


//            try {
//                telemetry.addData("X corner 1:", result.getDetectorResults().get(0).getTargetXPixels());
//            }
//            catch(Exception e){
//                telemetry.addLine("Catch");
//            }
//            telemetry.addData("X corner 1:", result.get);
//            telemetry.addData("Result: ",result);
//            telemetry.addData("result: ", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
            telemetry.update();
//            sleep(50);
        }

        limelight.stop();

    }

    private Point get_field_pos(Point centroid) {
        double theta = Math.toRadians(15);

        double x0 = 480 - 480;

        double x1 = 480 - centroid.y;
        double y = 320 - centroid.x;

        double x2 = 480 - 240;
        double x3 = 480 - 169;
        double f0 = 0;
        double f2 = 8.25;
        double f3 = 8.25+9.35;

        double k = (x2-x0)*(x3-x1)*(f3-f0)/(x3-x0)/(x2-x1)/(f2-f0);

        double cam_x = (k*f2 - f3)/(k-1);
        double cam_y = 0;

//        double field_x = cam_x*Math.cos(theta) + cam_y*Math.sin(theta);
//        double field_y = cam_y*Math.cos(theta) - cam_x*Math.sin(theta) + 5.5;

//        return new Point(field_x,field_y);
        return new Point(cam_x+6.75,cam_y);
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
//            double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;

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
}
//90-0.65
//0 - 1.8
//45 - 1.4

