package org.firstinspires.ftc.teamcode.Hardware;

import androidx.annotation.GuardedBy;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;



public class RobotHardware {

    //Todo Intake Actuators

    public  DcMotorEx lifter=null;

    public  Servo wrist=null;
    public  Servo arm=null;
    public  Servo gripper=null;
    public  Servo elbow=null;

    //Todo Sensor
    public RevColorSensorV3 ColorSensor=null;


    //Todo drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;
    public MecanumDrive drivetrain;
    public RevColorSensorV3 colorSensor;
    public DcMotorEx turret;


    //Todo SettingUp IMU
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;

    //Battery Voltage
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    //TODO ROBOT SETUP
    //////////ROBOT SETUP BEGINS/////////
    private static RobotHardware instance = null;    // ref variable to use robot hardware
    public boolean enabled;                          //boolean to return instance if robot is enabled.

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    /////////////ROBOT SETUP ENDS//////

    private HardwareMap hardwareMap;

    //Todo init() for hardware map
    //Call this method inside auto and teleop classes to instantly hardware map all actuators.
    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;

        //Map Intake Actuators
        lifter=hardwareMap.get(DcMotorEx.class, "motor1");
        wrist=hardwareMap.get(Servo.class,"wrist");
        arm=hardwareMap.get(Servo.class,"rotateGripper");
        turret = hardwareMap.get(DcMotorEx.class,"motor2");

        elbow=hardwareMap.get(Servo.class,"elbow");
        gripper=hardwareMap.get(Servo.class,"gripper");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        colorSensor.setGain(50);


        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        //DRIVE
        if(Globals.IS_CUSTOMDRIVE) {

            drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
            this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
            dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
            dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
            dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dtBackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
            dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dtFrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        //Map IMU
        if (Globals.IS_IMU) {
            synchronized (imuLock) {
                // Retrieve the IMU from the hardware map
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                imu.initialize(parameters);
            }
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }


    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuAngle = AngleUnit.normalizeRadians((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) + startOffset);
                    imuAngle= AngleUnit.normalizeRadians((-imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate)+startOffset);

                }
            }
        });
        imuThread.start();
    }

    public double getAngle() {
        return AngleUnit.normalizeRadians(imuAngle - imuOffset);
    }

    public void reset() {
        imuOffset = imuAngle;
//        imuOffset=AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + startOffset);
    }

    public double getVoltage() {
        return voltage;
    }
}
