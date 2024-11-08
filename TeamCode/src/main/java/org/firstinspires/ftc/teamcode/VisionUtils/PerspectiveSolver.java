package org.firstinspires.ftc.teamcode.VisionUtils;

import org.opencv.core.Point;

public class PerspectiveSolver {

    //Calculates coordinates with respect to bottom of camera frame using cross ratio
    //Input any 3 reference points in camera and their respective position in global frame

    /*Cross ratio = AC x BD / AD x BC, the variable input being B in this case
    We solve for field pos by equating the cross ratio as k = A'C' x B'D'/A'D' x B'C',
    where B' is the unknown.
    Final equation - k' = (AC x BD/AD x BC)*(A'D'/A'C') = (B'D'/B'C')*/

    double camera_angle;
    int CAMERA_HEIGHT;
    int CAMERA_WIDTH;
    double x_offset,y_offset;
    double cx1,cx2,cx3,fx1,fx2,fx3,w1,w2;
    double cy1,cy2,cy3,fy1,fy2,fy3;
    CameraOrientation orientation;


    public PerspectiveSolver(double camera_angle, double x_offset, double y_offset,
                             double cx1, double cx2, double cx3, double fx1, double fx2, double fx3,
                             double cy1, double cy2, double cy3, double fy1, double fy2, double fy3,
                             double w1, double w2, CameraOrientation orientation,
                             int CAMERA_HEIGHT, int CAMERA_WIDTH) {
        this.camera_angle = Math.toRadians(camera_angle);
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.cx1 = cx1;
        this.cx2 = cx2;
        this.cx3 = cx3;
        this.fx1 = fx1;
        this.fx2 = fx2;
        this.fx3 = fx3;
        this.cy1 = cy1;
        this.cy2 = cy2;
        this.cy3 = cy3;
        this.fy1 = fy1;
        this.fy2 = fy2;
        this.fy3 = fy3;
        this.w1 = w1;
        this.w2 = w2;
        this.CAMERA_HEIGHT = CAMERA_HEIGHT;
        this.CAMERA_WIDTH = CAMERA_WIDTH;
    }

    public Point getX2Y2(Point ObjectPose){
        //Solver for 2nd point in the points ordered A, B, C and D.

        double cam_x = CAMERA_HEIGHT - ObjectPose.y;
        double cam_y = CAMERA_WIDTH/2.0 - ObjectPose.x;

        double field_x,field_y;

        if (cam_x==cx2){
            field_x = fx2 + 7;
            //width is w1 at x1 and w2 at x2
            double width = w1 + (w2-w1)/(fx3-fx1)*(fx2-fx1);
            field_y = cam_y/CAMERA_WIDTH*width;
        }
        else{

            double k = ((cx2-cx1)*(cx3 - cam_x))/((cx3 - cx1)*(cx2 - cam_x))*(fx3-fx1)/(fx2-fx1);

            //
            field_x = (k*fx2-fx3)/(k-1) + 5;//1 inch offset from object center to the frame corner;
            double width = w1 + (w2-w1)/(fx2-fx1)*(field_x-fx1);
            field_y = cam_y/CAMERA_WIDTH*width;
        }

        double field_x_tr = field_x*Math.cos(camera_angle) + field_y*Math.sin(camera_angle) + x_offset;
        double field_y_tr = -field_x*Math.sin(camera_angle) + field_y*Math.cos(camera_angle) + y_offset-0.7;
        return new Point(field_x_tr,field_y_tr);
//        return new Point(field_x,field_y);
    }

}
