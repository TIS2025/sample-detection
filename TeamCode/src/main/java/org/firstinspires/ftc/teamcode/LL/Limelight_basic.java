package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "Limelight Basic Test")
public class Limelight_basic extends LinearOpMode {

    double currentRotation = 0;

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        Servo servo = hardwareMap.get(Servo.class,"wrist");
        servo.setPosition(0.5);
        double orientation = 0;

        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(4);

        waitForStart();
        limelight.start();
        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            if(gamepad1.a){
//                telemetry.addData("Result",result);
                try {
                    telemetry.addData("Result", result.getColorResults().get(0).getTargetCorners().get(0));
                    telemetry.addData("Result", result.getColorResults().get(0).getTargetCorners().get(1));
                    telemetry.addData("Result", result.getColorResults().get(0).getTargetCorners().get(2));
                    telemetry.addData("Result", result.getColorResults().get(0).getTargetCorners().get(3));
                    orientation = get_orientation(result);
                    telemetry.addData("Orientation",orientation);
                } catch (Exception e) {
                    telemetry.addLine("Failed to find samples");
                }
            }

            if(gamepad1.b){
//                servo.setPosition(orientation/180);
            }
            servo.setPosition(orientation/180);
            telemetry.update();
        }

//        limelight.stop();
    }

    public double getDirection(LLResult result) {
        List<List<Double>> points = result.getColorResults().get(0).getTargetCorners();
        double dx = points.get(0).get(0) - points.get(1).get(0);
        double dy = points.get(0).get(1) - points.get(1).get(1);
        double dx2 = points.get(0).get(0) - points.get(3).get(0);
        double dy2 = points.get(0).get(1) - points.get(3).get(1);
        double mag1 = dx*dx + dy*dy;
        double mag2 = dx2*dx2 + dy2*dy2;
        double new_rot;
        if (mag2 > mag1 * 1.5) new_rot = Math.toDegrees(Math.atan2(dx2, dy2));
        else new_rot = Math.toDegrees(Math.atan2(dx, dy));
        currentRotation = new_rot;
        return currentRotation;
    }

    public double get_orientation(LLResult result){
        List<List<Double>> res = result.getColorResults().get(0).getTargetCorners();
        double angle = 0;

        Point[] points = new Point[]{new Point(res.get(0).get(0), res.get(0).get(1)),
                new Point(res.get(1).get(0), res.get(1).get(1)),
                new Point(res.get(2).get(0), res.get(2).get(1)),
                new Point(res.get(3).get(0), res.get(3).get(1))
        };

            Arrays.sort(points, Comparator.comparingDouble(point -> point.y));
            Point tL = points[0];
            Point bR = points[3];

            Arrays.sort(points, Comparator.comparingDouble(point -> point.x));
            Point bL = points[0];
            Point tR = points[3];

            double w = calculateDistance(tL, tR);
            double h = calculateDistance(tL, bL);

            if (tL.y == tR.y) return 0;
            if (tL.y == bL.y) return 90;

            if (w >= h) angle = 90 - Math.toDegrees(Math.atan((tL.y - bL.y) / (tL.x - bL.x)));
            else angle = 90 - Math.toDegrees(Math.atan((tR.y - tL.y) / (tR.x - tL.x)));

        return angle;
    }
    private double calculateDistance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }
}
