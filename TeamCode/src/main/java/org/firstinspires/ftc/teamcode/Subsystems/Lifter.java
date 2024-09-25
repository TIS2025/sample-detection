package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Lifter {

    private final RobotHardware robot;
    private LifterState lifterState = LifterState.INIT;

    // Enum for Lifter states
    public enum LifterState {
        INIT,
        GROUND,
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR,
        LEVEL_FIVE,
        MAX_HEIGHT,
        SAFE
    }

    public Lifter(RobotHardware robot) {
        this.robot = robot;
    }

    // Update Lifter state
    public void updateLifterState(LifterState state) {
        this.lifterState = state;
        switch (state) {
            case INIT:
                setLifterPosition(Globals.lifterInit);
                break;
            case GROUND:
                setLifterPosition(Globals.lifterpick);
                break;
            case LEVEL_ONE:
                setLifterPosition(Globals.lifterOne);
                break;
            case LEVEL_TWO:
                setLifterPosition(Globals.lifterTwo);
                break;
            case LEVEL_THREE:
                setLifterPosition(Globals.lifterThree);
                break;
            case LEVEL_FOUR:
                setLifterPosition(Globals.lifterFour);
                break;
            case LEVEL_FIVE:
                setLifterPosition(Globals.lifterFive);
                break;
            case MAX_HEIGHT:
                setLifterPosition(Globals.lifterThree);
                break;
            case SAFE:
                setLifterPosition(Globals.lifterInit);
                break;
        }
    }

    // Set Lifter Position
    public void setLifterPosition(int position) {
        robot.lifter.setTargetPosition(position);
        robot.lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lifter.setPower(1.0); // Set appropriate power level
    }

    // Manual control for fine adjustments
    public void manualLifterControl(double power) {
        robot.lifter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.lifter.setPower(power);
    }

    // Stop the lifter motor
    public void stopLifter() {
        robot.lifter.setPower(0);
    }

    // Check if the lifter has reached its target position
    public boolean isLifterAtTarget() {
        return !robot.lifter.isBusy();
    }

    // Method to get the current position of the lifter
    public int getCurrentPosition() {
        return robot.lifter.getCurrentPosition();
    }

    // Methods for setting specific lifter actions
    public void liftToGround() {
        updateLifterState(LifterState.GROUND);
    }

    public void liftToLevelOne() {
        updateLifterState(LifterState.LEVEL_ONE);
    }

    public void liftToMaxHeight() {
        updateLifterState(LifterState.MAX_HEIGHT);
    }

    public int LifterPos(){
        return robot.lifter.getCurrentPosition();
    }
}
