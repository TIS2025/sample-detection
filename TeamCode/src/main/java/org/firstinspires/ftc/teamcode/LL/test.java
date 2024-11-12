package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo map")
public class test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo shoulder=hardwareMap.get(Servo.class,"ls");
        shoulder.setDirection(Servo.Direction.REVERSE);
        Servo wrist=hardwareMap.get(Servo.class,"wt");
        Servo grip=hardwareMap.get(Servo.class,"gp");
        Servo viper=hardwareMap.get(Servo.class,"vp");
        viper.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                shoulder.setPosition(0.5);
            }
            if(gamepad1.b){
                wrist.setPosition(0.5);
            }
            if(gamepad1.x){
                grip.setPosition(0.5);
            }
            if(gamepad1.y){
                viper.setPosition(0.5);
            }
        }
    }
}
