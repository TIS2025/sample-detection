package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Globals.GameConst;
import org.firstinspires.ftc.teamcode.Globals.VisionConst;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FieldAlignPipeline extends OpenCvPipeline {

    public double X;
    public double Y;
    public double Z = VisionConst.CAMERA_Z_OFFSET;
    public double pitch = VisionConst.CAMERA_PITCH;
    public double yaw = VisionConst.startPose.heading.real;
    public double roll = VisionConst.CAMERA_ROLL;

    public Point tl;
    public Point tr;
    public Point bl;
    public Point br;

    public Mat R;
    public Mat t;
    public Mat K;

    @Override
    public Mat processFrame(Mat input) {
        K = getIntrinsicMatrix();
        R = getRotationMatrix(pitch, yaw, roll);
        t = getTranslationMatrix(X, Y, Z);

        tl = calcProjection(GameConst.top_left, R, t, K);
        tr = calcProjection(GameConst.top_right, R, t, K);
        bl = calcProjection(GameConst.bottom_left, R, t, K);
        br = calcProjection(GameConst.bottom_right, R, t, K);

        Imgproc.line(input,tl,tr,new Scalar(0,255,0),3);
        Imgproc.line(input,tr,br,new Scalar(0,255,0),3);
        Imgproc.line(input,br,bl,new Scalar(0,255,0),3);
        Imgproc.line(input,bl,tl,new Scalar(0,255,0),3);

        K.release();
        R.release();
        t.release();

        return input;
    }

    public Point calcProjection(Point3 x, Mat rot, Mat trans, Mat K) {
        Mat X = Mat.zeros(3, 1, CvType.CV_64F);
        X.put(0, 0, x.x);
        X.put(1, 0, x.y);
        X.put(2, 0, x.z);

        Mat c_R_w = new Mat();
        Core.transpose(rot, c_R_w);

        Mat temp = new Mat();
        Core.multiply(c_R_w, new Scalar(-1), temp);

        Mat c_T_w = new Mat();
        Core.gemm(temp, trans, 1, new Mat(), 0, c_T_w);
        temp.release();

        Mat Rx = new Mat();
        Core.gemm(c_R_w, X, 1, new Mat(), 0, Rx);
        c_R_w.release();

        Mat cXw = new Mat();

        Core.add(Rx, c_T_w, cXw);
        Rx.release();
        c_T_w.release();

        Mat cx = Mat.zeros(3, 1, CvType.CV_64F);

        Core.divide(cXw, new MatOfDouble(cXw.get(2, 0)[0]), cx);
        cXw.release();

        Mat u = new Mat();
        Core.gemm(K, cx, 1, new Mat(), 0, u);
        cx.release();

        return new Point(Math.rint(u.get(0, 0)[0]), Math.rint(u.get(1, 0)[0]));
    }
    private Mat getIntrinsicMatrix() {
        Mat K = Mat.eye(3, 3, CvType.CV_64F);
        K.put(0, 0, VisionConst.fx);
        K.put(1, 1, VisionConst.fy);
        K.put(0, 2, VisionConst.cx);
        K.put(1, 2, VisionConst.cy);
        K.put(0, 1, VisionConst.s);

        return K;
    }
    private Mat getRotationMatrix(double pitch, double yaw, double roll) {
        double cp = Math.cos(Math.toRadians(pitch));
        double sp = Math.sin(Math.toRadians(pitch));
        double cy = Math.cos(Math.toRadians(yaw));
        double sy = Math.sin(Math.toRadians(yaw));
        double cr = Math.cos(Math.toRadians(roll));
        double sr = Math.sin(Math.toRadians(roll));

        Mat Rx = Mat.eye(3, 3, CvType.CV_64F);
        Rx.put(1, 1, cr);
        Rx.put(1, 2, -sr);
        Rx.put(2, 1, sr);
        Rx.put(2, 2, cr);

        Mat Ry = Mat.eye(3, 3, CvType.CV_64F);
        Ry.put(0, 0, cp);
        Ry.put(0, 2, sp);
        Ry.put(2, 0, -sp);
        Ry.put(2, 2, cp);

        Mat Rz = Mat.eye(3, 3, CvType.CV_64F);
        Rz.put(0, 0, cy);
        Rz.put(0, 1, -sy);
        Rz.put(1, 0, sy);
        Rz.put(1, 1, cy);

        Mat Rzy = new Mat();
        Core.gemm(Rz, Ry, 1, new Mat(), 0, Rzy);
        Mat R = new Mat();
        Core.gemm(Rzy, Rx, 1, new Mat(), 0, R);
        Rzy.release();
        Rx.release();
        Ry.release();
        Rz.release();

        return R;
    }
    private Mat getTranslationMatrix(double tx, double ty, double tz) {
        Mat t = Mat.zeros(3, 1, CvType.CV_64F);
        t.put(0, 0, tx);
        t.put(1, 0, ty);
        t.put(2, 0, tz);

        return t;
    }

    public void updatePose(double x, double y, double heading) {
        double cos = Math.cos(Math.toRadians(heading));
        double sin = Math.sin(Math.toRadians(heading));
        X = -x - VisionConst.CAMERA_X_OFFSET*cos + VisionConst.CAMERA_Y_OFFSET*sin;
        Y = y + VisionConst.CAMERA_X_OFFSET*sin + VisionConst.CAMERA_Y_OFFSET*cos;
        yaw = -heading+90;
    }
}
//
//
//
//    private Point calcProjection1(Point3 x,double[][] R, double[] t,double[][] K) {
//        double X = x.x;
//        double Y = x.y;
//        double Z = x.z;
//
//
//    }
//
//    private double[][] getIntrinsic(){
//        double[][] K = new double[3][3];
//        K[0][0] = VisionConst.fx;
//        K[1][1] = VisionConst.fy;
//        K[0][1] = VisionConst.s;
//        K[0][2] = VisionConst.cx;
//        K[1][2] = VisionConst.cy;
//        K[2][2] = 1;
//
//        return K;
//    }
//    private double[][] getRotation(double pitch,double yaw,double roll){
//
////        alpha - yaw, beta - pitch, gamma - roll
//        double alpha = Math.toRadians(yaw);
//        double beta = Math.toRadians(pitch);
//        double gamma = Math.toRadians(roll);
//
//        double cos_alpha = Math.cos(alpha);
//        double sin_alpha = Math.sin(alpha);
//        double cos_beta = Math.cos(beta);
//        double sin_beta = Math.sin(beta);
//        double cos_gamma = Math.cos(gamma);
//        double sin_gamma = Math.sin(gamma);
//
//        double[][] R = new double[3][3];
//
//        R[0][0] = cos_alpha*cos_beta;
//        R[0][1] = cos_alpha*sin_beta*sin_gamma - sin_alpha*cos_gamma;
//        R[0][2] = cos_alpha*sin_beta*cos_gamma + sin_alpha*sin_gamma;
//
//        R[1][0] = sin_alpha*cos_beta;
//        R[1][1] = sin_alpha*sin_beta*sin_gamma + cos_alpha*cos_gamma;
//        R[1][2] = sin_alpha*sin_beta*cos_gamma - cos_alpha*sin_gamma;
//
//        R[2][0] = -sin_beta;
//        R[2][1] = sin_alpha*cos_beta;
//        R[2][2] = cos_alpha*cos_beta;
//
//        return R;
//    }
//    private double[] getTranslation(double tx,double ty,double tz){
//        return new double[]{tx,ty,tz};
//    }
