package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Globals {

    public static boolean IS_IMU = false;
    public static boolean IS_CUSTOMDRIVE = false;
    public static double OpModeLoopTime = 0;

    // Gripper Positions
    public static double gripperOpen = 0.45;// grip out
    public static double gripperClose = 0.13;//grip in
    public static double gripperInit = 0.35;
    public static double gripperSafe = 0.3;
    public static double gripperIntermediate = 0.6;

    // Wrist Positions
    public static double wristInit = 0.55;
    public static double wristUp = 0;//horizontal
    public static double wristDown = 0.6;
    public static double wristSafe = 0.25;//-45 degree

    // Arm Positions
    public static double armInit = 0.4;
    public static double armUp = 0.75;
    public static double armDown = 0.25;
    public static double armMid = 0.5;
    public static double armSafe = 0.6;

    // Elbow Positions
    public static double elbowInit = 0.7;
    public static double elbowUp = 0.35;//aage
    public static double elbowDown = 0.7;//piche
    public static double elbowMid = 0.6;
    public static double elbowSafe = 0.7;

    //Zero servo  = 0
    //Servo pos 0.41 = 55.823
    public static double elbow0deg = 0.61;
    public static double elbow55_823deg = 0.41;



    public static int incVal = 27;
    public static int lifterInit = 0;
    public static int lifterOne = 120;
    public static int liftYellow = 200;
    public static int lifterTwo = 250; // 700
    public static int lifterThree = 880; // 800
    public static int lifterFour = 986;
    public static int lifterFive = 1100;
    public static int lifterpick= 80;
    public static int lifterSeven = 1600;
    public static int lifterEight = 1800;
    public static int lifterNine = 2100;

    //Gripper Arm values (for inverse kinematics)(inch)
    public static double elbow_1 = 7.2;
    public static double elbow_2 = 12;
    public static double offset_vertical = 1.5;
    public static double x_offset_center = 8.5;
    public static double center_offset_gripper = 2;

}
