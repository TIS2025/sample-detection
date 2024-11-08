package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //Rack and Pinion Arm Gripper
    public Servo shoulder = null;
    public Servo elbow = null;
    public Servo wrist = null;
    public Servo gripper = null;
    public CRServo rack = null;
    public DcMotorEx motor = null;

    public void init(HardwareMap hardwareMap){
        shoulder = hardwareMap.get(Servo.class,"shoulder");
        elbow = hardwareMap.get(Servo.class,"elbow");
        wrist = hardwareMap.get(Servo.class,"wrist");
        gripper = hardwareMap.get(Servo.class,"gripper");
        rack = hardwareMap.get(CRServo.class,"rack");

    }
}
