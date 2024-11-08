package org.firstinspires.ftc.teamcode.VisionUtils;

public enum CameraOrientation {
    UPRIGHT(1),
    UPSIDE_DOWN(0);
    public final int x;

    CameraOrientation(int x){this.x = x;}

    public int getOrientation(){return x;}

}
