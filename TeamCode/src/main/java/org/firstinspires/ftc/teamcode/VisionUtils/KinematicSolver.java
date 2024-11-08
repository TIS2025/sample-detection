package org.firstinspires.ftc.teamcode.VisionUtils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Point;

public class KinematicSolver {
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
            ext = target.x + x_offset - arm_l * Math.cos(Math.toRadians(theta - theta_offset));
        }
        ext = Math.max(0,Math.min(11,ext));
        theta = Math.max(-45,Math.min(45,theta));
        return new double[]{ext,theta};
    }
}
