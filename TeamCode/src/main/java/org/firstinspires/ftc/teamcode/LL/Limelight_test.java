package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

import java.util.List;


@TeleOp(name = "Limelight")
public class Limelight_test extends LinearOpMode {
    public static Limelight3A limelight;
    public double[] pythonOutputs = null;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);

//        limelight.reloadPipeline();
        limelight.pipelineSwitch(1);

//        limelight.stop();
        while(opModeInInit()){
            if(!limelight.isConnected()){
//                limelight.setPollRateHz(100);
                limelight.stop();
                sleep(1000);
                limelight.start();
            }


            telemetry.addData("Connected",limelight.isConnected());
            telemetry.addData("Connected",limelight.isRunning());
            telemetry.update();
        }
//        limelight.start();
//        Thread.sleep(100);
        waitForStart();

        while (opModeIsActive()) {

//            sleep(20);
//            LLResult result = limelight.getLatestResult();
            LLResult result = limelight.getLatestResult();

//            telemetry.addData("Valid: ", result.isValid());


            try{
                telemetry.addData("result", result.getDetectorResults().get(0).getTargetCorners());


                List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
                List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
                List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
                List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;

                Point centroid = new Point(cx, cy);

                telemetry.addData("centroid", new Point(cx, cy));

                telemetry.addData("Field_Pos", get_field_pos(centroid));
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
}

