package org.firstinspires.ftc.teamcode.VisionUtils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class LimeLightUtils {
    public Limelight3A limelight3A;
    public LimeLightUtils(HardwareMap hardwareMap){
        this.limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
    }

    List<Sample> red_samples = new ArrayList<>();
    List<Sample> yellow_samples = new ArrayList<>();
    List<Sample> blue_samples = new ArrayList<>();
    List<Sample> yellow_and_blue_samples = new ArrayList<>();
    List<Sample> yellow_and_red_samples = new ArrayList<>();
    PerspectiveSolver perspectiveSolver = new PerspectiveSolver();

    public static void filter_yellow(LLResult result){
        try {
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

                double Area = width * height;
                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = Psolver.getX2Y2(new Point(cx, cy));
                boolean orientation = width / height > whRatio;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();

//                if(field_pos.x >10 && orientation){
//                    extension_offFalseX = field_pos.x*0.05;
//                }
//                else{
//                    extension_offFalseX = 1.0;
//                }
                double offset = orientation ? extension_offTrueX : extension_offFalseX;
                double offset_y = orientation ? extension_offTrueY : extension_offFalseY;
                if ((field_pos.y < Ypositive && field_pos.y > Ynegative) && (field_pos.x < Xhigh && field_pos.x > XLow) && (class_id == 0 || class_id == 2) ) {
                    field_pos.x += offset;
                    field_pos.y += offset_y;
                    relevantSamples.add(new Sample(field_pos, class_id, confidence, orientation, width/height));
                }

//                relevantSamples.removeIf(o -> (o.whRatio <= 1.5 && o.whRatio >= 0.95));
//                relevantSamples.removeIf(o -> !( (o.whRatio >= 0.5 && o.whRatio <= 1.15) || (o.whRatio >= 1.2 && o.whRatio <= 1.65) ));

//                Comparator<Sample> comparator = Comparator.comparingDouble(o->Math.abs(o.field_pos.y));
//                comparator = comparator.thenComparingDouble(o->Math.abs(o.field_pos.x));
//                relevantSamples.sort(comparator);

            }
        } catch (Exception e) {
            telemetry.addLine("Failed to input samples");
        }
    }
    public static void filter_red(){

    }
    public static void filter_blue(){

    }
    public static void filter_yellow_red(){

    }
    public static void filter_yellow_blue(){

    }


    public static class PerspectiveSolver{

        //Calculates coordinates with respect to bottom of camera frame using cross ratio
        //Input any 3 reference points in camera and their respective position in global frame

        /*Cross ratio = AC x BD / AD x BC, the variable input being B in this case
        We solve for field pos by equating the cross ratio as k = A'C' x B'D'/A'D' x B'C',
        where B' is the unknown.
        Final equation - k' = (AC x BD/AD x BC)*(A'D'/A'C') = (B'D'/B'C')*/

        double camera_angle;
        int CAMERA_HEIGHT;
        int CAMERA_WIDTH;
        double x_offset,y_offset,camera_offset;
        double cx1,cx2,cx3,fx1,fx2,fx3,w1,w2;
        double cy1,cy2,cy3,fy1,fy2,fy3;
        org.firstinspires.ftc.teamcode.VisionUtils.CameraOrientation orientation;


        public PerspectiveSolver() {
            this.camera_angle = Math.toRadians(VisionConst.camera_angle);
            this.x_offset = VisionConst.x_offset;
            this.y_offset = VisionConst.y_offset;
            this.camera_offset = VisionConst.camera_offset;
            this.cx1 = VisionConst.cx1;
            this.cx2 = VisionConst.cx2;
            this.cx3 = VisionConst.cx3;
            this.fx1 = VisionConst.fx1;
            this.fx2 = VisionConst.fx2;
            this.fx3 = VisionConst.fx3;
            this.cy1 = VisionConst.cy1;
            this.cy2 = VisionConst.cy2;
            this.cy3 = VisionConst.cy3;
            this.fy1 = VisionConst.fy1;
            this.fy2 = VisionConst.fy2;
            this.fy3 = VisionConst.fy3;
            this.w1 = VisionConst.w1;
            this.w2 = VisionConst.w2;
            this.CAMERA_HEIGHT = VisionConst.CAMERA_HEIGHT;
            this.CAMERA_WIDTH = VisionConst.CAMERA_WIDTH;
        }

        public Point getX2Y2(Point ObjectPose){
            //Solver for 2nd point in the points ordered A, B, C and D.

            double cam_x = CAMERA_HEIGHT - ObjectPose.y;
            double cam_y = CAMERA_WIDTH/2.0 - ObjectPose.x;

            double field_x,field_y;

            if (cam_x==cx2){
                field_x = fx2 + camera_offset;
                //width is w1 at x1 and w2 at x2
                double width = w1 + (w2-w1)/(fx2-fx1)*(fx2-fx1);
                field_y = cam_y/CAMERA_WIDTH*width;
            }
            else{

                double k = ((cx2-cx1)*(cx3 - cam_x))/((cx3 - cx1)*(cx2 - cam_x))*(fx3-fx1)/(fx2-fx1);

                //
                field_x = (k*fx2-fx3)/(k-1) + camera_offset;//1 inch offset from object center to the frame corner;
                double width = w1 + (w2-w1)/(fx2-fx1)*(field_x-fx1);
                field_y = cam_y/CAMERA_WIDTH*width;
            }

            double field_x_tr = field_x*Math.cos(camera_angle) + field_y*Math.sin(camera_angle) + x_offset;
            double field_y_tr = -field_x*Math.sin(camera_angle) + field_y*Math.cos(camera_angle) + y_offset;
            return new Point(field_x_tr,field_y_tr);
        }
    }
    public static class KinematicSolver {
        double arm_l1;
        double arm_l2;
        double x_offset;
        double y_offset;
        double theta_offset;

        public KinematicSolver(double arm_l1,double arm_l2,double x_offset,double y_offset){
            this.arm_l1 = arm_l1;
            this.arm_l2 = arm_l2;
            this.x_offset = x_offset;
            this.y_offset = y_offset;
            this.theta_offset = Math.atan(arm_l1/arm_l2);
        }

        //takes target point as input and gives extension in inches and yaw in degrees as element 0 and 1 of the array
        public double[] getExtYaw(Point target){
            double theta = 0;
            double ext = 0;
            double arm_l = Math.sqrt(arm_l1*arm_l1 + arm_l2*arm_l2);

            if(Math.abs(target.y+y_offset)<=arm_l) {
                theta = Math.toDegrees(Math.asin((target.y + y_offset) / arm_l) + theta_offset);
                ext = target.x + x_offset - arm_l;
//            ext = target.x + x_offset - arm_l * Math.cos(Math.toRadians(theta - theta_offset));
            }
            ext = Math.max(0,Math.min(11,ext));
            theta = Math.max(-45,Math.min(45,theta));
//        return new double[]{ext,theta};
            return new double[]{ext,0};
        }
    }
    public static class Sample{
        public double confidence;
        public int class_id;
        public Point field_pos;
        public boolean orientation;
        public double wh_ratio;

        public Sample(Point field_pos,int class_id,double confidence,boolean orientation,double wh_ratio){
            this.field_pos = field_pos;
            this.class_id = class_id;
            this.confidence = confidence;
            this.orientation = orientation;
            this.wh_ratio = wh_ratio;
        }
    }
    public enum CameraOrientation {
        UPRIGHT(0),
        UPSIDE_DOWN(1),
        RIGHT_90(2),
        LEFT_90(3);
        public final int x;

        CameraOrientation(int x){this.x = x;}

        public int getOrientation(){return x;}

    }
}
