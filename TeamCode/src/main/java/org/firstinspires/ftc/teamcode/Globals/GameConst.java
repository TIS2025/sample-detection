package org.firstinspires.ftc.teamcode.Globals;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Point3;

@Config
public class GameConst {
    public static double FIELD_WIDTH = 27.5;
    public static double FIELD_HEIGHT = 42.75;

    public static Point3 top_left = new Point3(FIELD_WIDTH/2,FIELD_HEIGHT/2,0);
    public static Point3 top_right = new Point3(FIELD_WIDTH/2,-FIELD_HEIGHT/2,0);
    public static Point3 bottom_left = new Point3(-FIELD_WIDTH/2,FIELD_HEIGHT/2,0);
    public static Point3 bottom_right = new Point3(-FIELD_WIDTH/2,-FIELD_HEIGHT/2,0);


}
