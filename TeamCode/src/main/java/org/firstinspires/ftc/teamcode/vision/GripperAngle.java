package org.firstinspires.ftc.teamcode.vision;

public enum GripperAngle {
    ANGLE_0(0),
    ANGLE_45(45),
    ANGLE_90(90),
    ANGLE_135(135);
    public final int angle;
    GripperAngle(int angle) {
        this.angle = angle;
    }

    // Getter for value
    public int getValue() {
        return angle;
    }
}