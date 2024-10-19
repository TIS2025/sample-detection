package org.firstinspires.ftc.teamcode.Globals;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class VisionConst {

    public static double CAMERA_X_OFFSET = 7.5;
    public static double CAMERA_Y_OFFSET = 5;
    public static double CAMERA_Z_OFFSET = 6.2;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_ROLL = 78;
    public static double CAMERA_YAW = 0;

    public static Pose2d startPose = new Pose2d(0,-63,90);

    //Camera Calibration
    public static double fx = 631.257;
    public static double fy = 631.257;
    public static double cx = 645.384;
    public static double cy = 372.677;
    public static double s = 0;
}
