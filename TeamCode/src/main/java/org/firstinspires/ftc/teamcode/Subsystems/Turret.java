package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Turret {
    private final RobotHardware robot;

    public Turret(RobotHardware robot){
        this.robot=robot;
    }

    public void setTurretPosition(int position){
        robot.turret.setTargetPosition(position);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.8);
    }

    public void stopTurret() {
        robot.turret.setPower(0);
    }

    public boolean isTurretAtTarget() {
        return !robot.turret.isBusy();
    }

    public int TurretPos(){
        return robot.turret.getCurrentPosition();
    }

}
