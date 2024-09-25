package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "MotorServoTest")
public class MotorServoTest extends LinearOpMode {

    DcMotorEx motor1;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    Servo servo5;
    Servo servo6;

    double MotorPower = 0;
    double servoPos = 0.5;
    Gamepad C = new Gamepad();
    Gamepad P = new Gamepad();


    @Override

    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class,"motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo1 = hardwareMap.get(Servo.class,"servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
//        servo4 = hardwareMap.get(Servo.class,"servo4");
//        servo5 = hardwareMap.get(Servo.class,"servo5");
//        servo6 = hardwareMap.get(Servo.class,"servo6");

        servo1.setPosition(servoPos);


        waitForStart();
        while(opModeIsActive()){
            P.copy(C);
            C.copy(gamepad1);

            if(C.left_bumper && !P.left_bumper) MotorPower+=0.1;
            if(C.right_bumper && !P.right_bumper) MotorPower-=0.1;
            if(C.dpad_up && !P.dpad_up) servoPos+=0.05;
            if(C.dpad_down && !P.dpad_down) servoPos-=0.05;

            if(C.a) MotorPower = 0;

            if(C.x && !P.x) servoPos+=0.01;
            if(C.y && !P.y) servoPos-=0.01;


            motor1.setPower(MotorPower);

            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);
            servo3.setPosition(servoPos);
//            servo4.setPosition(servoPos);
//            servo5.setPosition(servoPos);
//            servo6.setPosition(servoPos);

            telemetry.addData("Motor Power",motor1.getPower());
            telemetry.addData("Servo Pos",servo1.getPosition());
            telemetry.addData("Current",motor1.getCurrent(CurrentUnit.MILLIAMPS))   ;
            telemetry.update();


        }

    }
}
