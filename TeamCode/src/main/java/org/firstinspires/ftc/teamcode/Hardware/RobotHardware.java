package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public Servo leftShoulder;
    public Servo rightShoulder;
    public Servo elbow;
    public Servo wrist;
    public RevColorSensorV3 colorSensor;

    public RobotHardware(HardwareMap hardwareMap){
        this.leftShoulder = hardwareMap.get(Servo.class,"lsh");
        this.rightShoulder = hardwareMap.get(Servo.class,"rsh");
        this.elbow = hardwareMap.get(Servo.class,"elb");
        this.wrist = hardwareMap.get(Servo.class,"wr");
    }
}
