package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Gripper {
    RobotHardware robot;

    public enum ShoulderState{
        NEUTRAL,FRONT,BACK
    }
    ShoulderState shoulderState = ShoulderState.NEUTRAL;

    public enum ElbowState{
        NEUTRAL,FRONT,BACK
    }
    ElbowState elbowState = ElbowState.NEUTRAL;

    public enum WristState{
        ROT0,
        ROT90,
        ROT45
    }
    WristState wristState = WristState.ROT45;

    public void SetShoulder(ShoulderState state){
        switch(state){
            case NEUTRAL:
                robot.leftShoulder.setPosition(ServoConst.shoulder_neutral);
                robot.rightShoulder.setPosition(1-ServoConst.shoulder_neutral);
                shoulderState = ShoulderState.NEUTRAL;
                break;
            case BACK:
                robot.leftShoulder.setPosition(ServoConst.shoulder_backward);
                robot.rightShoulder.setPosition(1-ServoConst.shoulder_backward);
                shoulderState = ShoulderState.BACK;
                break;
            case FRONT:
                robot.leftShoulder.setPosition(ServoConst.shoulder_forward);
                robot.rightShoulder.setPosition(1-ServoConst.shoulder_forward);
                shoulderState = ShoulderState.FRONT;
                break;
        }
    }
    public void SetElbow(ElbowState state){
        switch(state){
            case NEUTRAL:
                robot.elbow.setPosition(0.5);
                elbowState = ElbowState.NEUTRAL;
                break;
            case BACK:
                robot.elbow.setPosition(0.3);
                elbowState = ElbowState.BACK;
                break;
            case FRONT:
                robot.elbow.setPosition(0.7);
                elbowState = ElbowState.FRONT;
                break;
        }
    }
    public void SetWrist(WristState state){
        switch(state){
            case ROT45:
                robot.wrist.setPosition(0.5);
                wristState = WristState.ROT45;
                break;
            case ROT0:
                robot.wrist.setPosition(0.3);
                wristState = WristState.ROT0;
                break;
            case ROT90:
                robot.wrist.setPosition(0.7);
                wristState = WristState.ROT90;
                break;
        }
    }

    public InstantAction ShoulderAction(ShoulderState state){
        return new InstantAction(()->SetShoulder(state));
    }

    public InstantAction ElbowAction(ElbowState state){
        return new InstantAction(()->SetElbow(state));
    }

    public InstantAction WristAction(WristState state){
        return new InstantAction(()->SetWrist(state));
    }
}
