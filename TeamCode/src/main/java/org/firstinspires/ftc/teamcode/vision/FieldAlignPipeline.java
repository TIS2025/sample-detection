package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FieldAlignPipeline extends OpenCvPipeline {

    public double X = 0;
    public double Y = 56;

    double[] cx1 = new double[]{19.85185027,-0.024658631,-0.136692177,-9.248626928,0.110434036,979.2620497};
    double[] cy1 = new double[]{-1.223870156,-0.000532753,0.021489861,-3.096258706,0.014748503,709.6236845};
    double[] cx2 = new double[]{20.74987679,-0.014963739,-0.150122477,-30.33013852,0.267267134,2021.081656};
    double[] cy2 = new double[]{-1.2209666,-0.000179572,0.021386953,-3.177040603,0.0156009,711.4673157};
    double[] cx3 = new double[]{94.685297,-0.116149723,-1.205402988,-140.1494187,1.239093548,5334.850732};
    double[] cy3 = new double[]{-1.567587354,-0.002160235,0.030826241,-101.1118968,0.874538362,3667.66948};
    double[] cx4 = new double[]{89.36346847,0.036265645,-1.153074495,119.3365929,-1.008327177,-2963.740717};
    double[] cy4 = new double[]{0.848097513,-0.00492661,-0.014197487,-84.09760189,0.708937908,3233.286541};

    Point[] field = new Point[4];
    @Override
    public Mat processFrame(Mat input) {
        field = get_field(X,Y);
//        Imgproc.polylines(input,field,true,new Scalar(0,255,0),3);
        Imgproc.line(input,field[0],field[1],new Scalar(0,255,0),3);
        Imgproc.line(input,field[1],field[2],new Scalar(0,255,0),3);
        Imgproc.line(input,field[2],field[3],new Scalar(0,255,0),3);
        Imgproc.line(input,field[3],field[0],new Scalar(0,255,0),3);
//        input.release();
        return input;
    }

    public void updateXY(double x,double y){
        X = x;
        Y = 56-(y-56);
    }

    public Point[] get_field(double x,double y){
        Point tl = new Point(curve_func(x,y,cx1),curve_func(x,y,cy1));
        Point tr = new Point(curve_func(x,y,cx2),curve_func(x,y,cy2));
        Point br = new Point(curve_func(x,y,cx3),curve_func(x,y,cy3));
        Point bl = new Point(curve_func(x,y,cx4),curve_func(x,y,cy4));

        return new Point[]{tl,tr,br,bl};
    }
    private int curve_func(double x,double y,double[] c){
        return (int)((c[0]*x + c[1]*x*x + c[2]*x*y + c[3]*y + c[4]*y*y + c[5])/1.5);
    }


}
