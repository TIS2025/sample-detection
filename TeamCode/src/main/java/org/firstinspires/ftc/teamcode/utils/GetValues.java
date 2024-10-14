package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@TeleOp(name = "Get Values")
public class GetValues extends LinearOpMode {
    public double shoulder_pos = 0.5;
    public double wrist_pos = 0.5;
    public double elbow_pos = 0.5;
    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.leftShoulder.setPosition(shoulder_pos);
        robot.rightShoulder.setPosition(1-shoulder_pos);
        robot.wrist.setPosition(wrist_pos);
        robot.elbow.setPosition(elbow_pos);

        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();

        waitForStart();
        while (opModeIsActive()){
            P.copy(C);
            C.copy(gamepad1);

            if(C.left_bumper && !P.left_bumper && shoulder_pos<=1) shoulder_pos+=0.02;
            if(C.right_bumper && !P.right_bumper && shoulder_pos>=0) shoulder_pos-=0.02;
            robot.leftShoulder.setPosition(shoulder_pos);
            robot.rightShoulder.setPosition(1-shoulder_pos);

            if(C.a && !P.a && elbow_pos<=1) elbow_pos+=0.02;
            if(C.b && !P.b && elbow_pos>=0) elbow_pos-=0.02;
            robot.elbow.setPosition(elbow_pos);

            if(C.dpad_up && !P.dpad_up && wrist_pos<=1) wrist_pos+=0.02;
            if(C.dpad_down && !P.dpad_down && wrist_pos>=0) wrist_pos-=0.02;
            robot.wrist.setPosition(wrist_pos);

            telemetry.addData("Wrist pos","%.2f",robot.wrist.getPosition());
            telemetry.addData("Elbow","%.2f",robot.elbow.getPosition());
            telemetry.addData("Shoulder pos","%.2f",robot.leftShoulder.getPosition());
            telemetry.update();
        }
    }
}
