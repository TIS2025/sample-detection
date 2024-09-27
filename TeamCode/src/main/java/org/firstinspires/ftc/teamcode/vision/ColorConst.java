package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class ColorConst {

    //Color thresholds
    public static Scalar highRed = new Scalar(130, 255, 255);
    public static Scalar lowRed = new Scalar(120, 70, 50);
    public static Scalar highBlue = new Scalar(30, 255, 255);
    public static Scalar lowBlue = new Scalar(0, 50, 30);//70,50
    public static Scalar highYellow = new Scalar(100, 255, 255);
    public static Scalar lowYellow = new Scalar(90, 70, 50);

//    public static Scalar highRed = new Scalar(0,0,0);
//    public static Scalar lowRed = new Scalar(0,0,0);
//    public static Scalar highBlue = new Scalar(0,0,0);
//    public static Scalar lowBlue = new Scalar(0,0,0);
//    public static Scalar highYellow = new Scalar(0,0,0);
//    public static Scalar lowYellow = new Scalar(0,0,0);
}
