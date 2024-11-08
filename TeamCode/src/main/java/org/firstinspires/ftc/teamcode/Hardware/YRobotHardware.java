package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class YRobotHardware {

    //TODO SERVO
    public Servo shoulder=null;
    public Servo wrist=null;
    public Servo grip=null;
    public Servo viper=null;

    //TODO MOTOR
    public DcMotorEx lifterL=null;
    public DcMotorEx lifterR=null;
    public DcMotorEx liftChanger=null;
    public DcMotorEx lowHang=null;
    public RevColorSensorV3 color=null;

    // Static instance to be used across all instances
    private static YRobotHardware instance;
    public boolean enabled;
    private HardwareMap hardwareMap;  // Linking to hardware map with robot hardware.


    public static YRobotHardware getInstance() {
        if (instance == null) {
            instance = new YRobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        //INTAKE SERVOS
        shoulder=hardwareMap.get(Servo.class,"ls");
        wrist=hardwareMap.get(Servo.class,"wt");
        grip=hardwareMap.get(Servo.class,"gp");
        viper=hardwareMap.get(Servo.class,"vp");

        //ELEVATOR
        lifterL=hardwareMap.get(DcMotorEx.class,"ll");
        lifterR=hardwareMap.get(DcMotorEx.class,"lr");

        //HIGH-HANG  / ROTATE
        liftChanger=hardwareMap.get(DcMotorEx.class,"lc");
        //LOW-HANG / REACH
        lowHang=hardwareMap.get(DcMotorEx.class,"lHang");
        color=hardwareMap.get(RevColorSensorV3.class,"color");

        lifterL.setDirection(DcMotorSimple.Direction.REVERSE);

    }


}
