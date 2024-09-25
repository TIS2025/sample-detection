package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Intake {

    public RobotHardware robot;

    // Enums for different component states
    public GripperState gripperState = GripperState.INIT;
    public ArmState armState = ArmState.INIT;
    public WristState wristState = WristState.INIT;
    public ElbowState elbowState = ElbowState.INIT;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }

    // Enums for Gripper states
    public enum GripperState {
        INIT,
        OPEN,
        CLOSE,
        SAFE
    }

    // Enums for Arm states
    public enum ArmState {
        INIT,
        UP,
        DOWN,
        MID,
        SAFE
    }

    // Enums for Wrist states
    public enum WristState {
        INIT,
        UP,
        DOWN,
        MID,
        SAFE
    }

    // Enums for Elbow states
    public enum ElbowState {
        INIT,
        UP,
        DOWN,
        MID,
        SAFE
    }

    // Update Gripper state
    public void updateGripperState(GripperState state) {
        this.gripperState = state;
        switch (state) {
            case INIT:
                setGripper(Globals.gripperInit);
                break;
            case OPEN:
                setGripper(Globals.gripperOpen);
                break;
            case CLOSE:
                setGripper(Globals.gripperClose);
                break;
            case SAFE:
                setGripper(Globals.gripperSafe);
                break;
        }
    }

    // Update Arm state
    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (state) {
            case INIT:
                setArm(Globals.armInit);
                break;
            case UP:
                setArm(Globals.armUp);
                break;
            case DOWN:
                setArm(Globals.armDown);
                break;
            case MID:
                setArm(Globals.armMid);
                break;
            case SAFE:
                setArm(Globals.armSafe);
                break;
        }
    }

    // Update Wrist state
    public void updateWristState(WristState state) {
        this.wristState = state;
        switch (state) {
            case INIT:
                setWrist(Globals.wristInit);
                break;
            case UP:
                setWrist(Globals.wristUp);
                break;
            case DOWN:
                setWrist(Globals.wristDown);
                break;

            case SAFE:
                setWrist(Globals.wristSafe);
                break;
        }
    }

    // Update Elbow state
    public void updateElbowState(ElbowState state) {
        this.elbowState = state;
        switch (state) {
            case INIT:
                setElbow(Globals.elbowInit);
                break;
            case UP:
                setElbow(Globals.elbowUp);
                break;
            case DOWN:
                setElbow(Globals.elbowDown);
                break;
            case MID:
                setElbow(Globals.elbowMid);
                break;
            case SAFE:
                setElbow(Globals.elbowSafe);
                break;
        }
    }

    // Methods to set positions for different components
    public void setGripper(double position) {
        robot.gripper.setPosition(position);
    }

    public void setArm(double position) {
        robot.arm.setPosition(position);
    }

    public void setWrist(double position) {
        robot.wrist.setPosition(position);
    }

    public void setElbow(double position) {
        robot.elbow.setPosition(position);
    }

    public double elbowPos(){ return robot.elbow.getPosition();}
}
