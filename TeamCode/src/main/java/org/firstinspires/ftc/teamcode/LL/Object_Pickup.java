package org.firstinspires.ftc.teamcode.LL;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.YRobotHardware;
import org.firstinspires.ftc.teamcode.InterTest3;
import org.firstinspires.ftc.teamcode.VisionUtils.CameraOrientation;
import org.firstinspires.ftc.teamcode.VisionUtils.PerspectiveSolver;
import org.opencv.core.Point;

import java.util.List;

@TeleOp(name = "Limelight pickup")
@Config
public class Object_Pickup extends LinearOpMode {

    public static Limelight3A limelight;
    YRobotHardware robot;

    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;

    double w1 = 8.2;
    double w2 = 17.75;
    double cx1 = CAMERA_HEIGHT - 480;
    double cx2 = CAMERA_HEIGHT - 255;
    double cx3 = CAMERA_HEIGHT - 182;



    double angle = 16;
    double cam_offset = 7;
    double x_offset = 1;
    double y_offset = 5;

    PerspectiveSolver solver = new PerspectiveSolver(angle,x_offset,y_offset,cx1,cx2,cx3,0,10,20,
            0,0,0,0,0,0,w1,w2, CameraOrientation.UPRIGHT,CAMERA_HEIGHT,CAMERA_WIDTH);

    //LINEAR-INTERPOLATION VALUES
    public static int minEncoderValue = 300;   //500
    public static int maxEncoderValue = 1500;  // Max slider extension
    public static double minServoPosition = 0.4;//0.45
    public static double maxServoPosition = 0.6;  // Range from 0.0 to 1.0 (or any range appropriate for your servo)

    public static double k = 0.05;

    //ENCODER TO X-DISTANCE
    public static int offsetDist=0;
    public static double PULLEY_RADIUS = 0.9;
    public static final int TICKS_PER_REV = 28;
    public static double GEAR_RATIO = 0.066;

    public static int elevatorPos=0;

    //VARIABLE VALUES
    public static double gripOpen=0.426;
    public static double gripClose=  0.589;
    public static double shouldrPrePicke=  0.432;
    public static double shouldePick=  0.397;

    RobotState robotState = RobotState.INIT;
    GripperState gripperState = GripperState.OPEN;
    WristState wristState= WristState.HORIZONTAL;
    ViperState viperState= ViperState.INIT;
    ShoulderState shoulderState= ShoulderState.INIT;

    public double obj_pos = 0;
    public double prev_obj_pos=0;
    Point field_pos,centroid;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetry.setMsTransmissionInterval(100);
        robot = YRobotHardware.getInstance();



//        limelight.reloadPipeline();
        limelight.pipelineSwitch(1);

        while(opModeInInit()){
            if(!limelight.isConnected()){
//                limelight.setPollRateHz(100);
                limelight.stop();
                sleep(1000);
                limelight.start();
            }


            telemetry.addData("Connected",limelight.isConnected());
            telemetry.addData("Connected",limelight.isRunning());

//            init_bot();
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            interpret_limelight(result);
            telemetry.update();
        }
    }

    private void init_bot() {
        extendTo(0,1);
        extendTurret(0,0.4);
        sleep(300);
        wrist(WristState.HORIZONTAL);
        viper(ViperState.INIT);
        shoulder(ShoulderState.INIT);
        sleep(250);
        extendTurret(0,0.5);
    }
    public void interpolate_shoulder(){
        int currentEncoderValue = robot.lifterL.getCurrentPosition();//
        double servoPosition = minServoPosition + ((double)(currentEncoderValue - minEncoderValue) / (maxEncoderValue - minEncoderValue)) * (maxServoPosition - minServoPosition);
        robot.shoulder.setPosition(servoPosition);
    }
    public void extendTurret(int Target,double pow){
        robot.liftChanger.setTargetPosition(Target);
        robot.liftChanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftChanger.setPower(pow);
    }

    public void extendTo(int Target,double pow){
        robot.lifterL.setTargetPosition(Target);
        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterL.setPower(pow);
        robot.lifterR.setTargetPosition(Target);
        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterR.setPower(pow);
    }
    public void shoulder(ShoulderState state){
        shoulderState=state;
        switch (state){
            case PICK:
                robot.shoulder.setPosition(shouldePick);
                break;
            case PREPICK:
                robot.shoulder.setPosition(shouldrPrePicke);
                break;
            case DROP:
                robot.shoulder.setPosition(1);
                break;
            case INIT:  //IDLE STATE
                robot.shoulder.setPosition(1);
                break;
            case FOLLOWER:
                break;
            case NOTFOLLOWER:
                break;
        }

    }
    public void gripper(GripperState state){
        gripperState=state;
        switch (gripperState) {
            case OPEN:
                robot.grip.setPosition(gripOpen);
                break;
            case CLOSE:
                robot.grip.setPosition(gripClose);
                break;
        }
    }
    public void wrist(WristState state){
        wristState=state;
        switch (wristState){
            case VERT:
                robot.wrist.setPosition(0.42);
                break;
            case HORIZONTAL:
                robot.wrist.setPosition(0.843 );
                break;
        }
    }
    public void viper(ViperState state){
        viperState=state;
        switch (state){
            case INIT:
                robot.viper.setPosition(0.178);
                break;
            case PICK:
                robot.viper.setPosition(0.178);
                break;
            case DROP:
                robot.viper.setPosition(0.868);
                break;
        }
    }

    public void interpret_limelight(LLResult result){

        try{
            List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
            List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
            List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
            List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);

            double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
            double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
//            double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;

            centroid = new Point(cx, cy);
            field_pos = solver.getX2Y2(centroid);
        }
        catch (Exception e){
            field_pos = new Point(0,0);
        }
        telemetry.addData("field_pos",field_pos);
    }


    public enum ShoulderState{
        PICK,
        PREPICK,
        DROP,
        INIT,
        FOLLOWER,
        NOTFOLLOWER,
    }
    public enum GripperState{
        OPEN,
        CLOSE,
    }
    public enum WristState {
        HORIZONTAL,
        VERT,
    }
    public enum ViperState {
        INIT,
        PICK,
        DROP,
    }
    public enum RobotState {
        INIT,
        PICK,
        DROP,
        FOLLOWER,
        GRIPPER,
    }
}
