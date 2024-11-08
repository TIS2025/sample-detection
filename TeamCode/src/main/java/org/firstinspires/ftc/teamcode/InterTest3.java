package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.YRobotHardware;
import org.opencv.core.Point;

import java.util.List;

@TeleOp
@Config
public class InterTest3 extends LinearOpMode {

    public static Limelight3A limelight;
    YRobotHardware robot=YRobotHardware.getInstance();

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

    public double obj_pos = 0;
    public double prev_obj_pos=0;





    RobotState robotState = RobotState.INIT;
    GripperState gripperState = GripperState.OPEN;
    WristState wristState= WristState.HORIZONTAL;
    ViperState viperState= ViperState.INIT;
    ShoulderState shoulderState= ShoulderState.INIT;

    //FLAGS
    boolean toggle = false;
    boolean previousButtonState = false;
    MecanumDrive drive;

    public void printStuff(){
        telemetry.addData("wristState",wristState);
        telemetry.addData("ViperState",viperState);
        telemetry.addData("gripperState",gripperState);
        telemetry.addLine("CURRENT POS");
        telemetry.addData("LIFTER L POS",encoderTicksToInches(robot.lifterL.getCurrentPosition()));
        telemetry.addData("Servo Actual GETPosition: %5.2f", robot.shoulder.getPosition());
        telemetry.addData("LIFTER R POS",robot.lifterR.getCurrentPosition());
        telemetry.addData("TURRET R POS",robot.liftChanger.getCurrentPosition());
        telemetry.addData("Obj pos",InchesToEncoderTicks(obj_pos));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        drive=new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);

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

            INIT();
            stateBased(RobotState.INIT);

            telemetry.update();
        }



        waitForStart();
        while (opModeIsActive()){

            //TODO: LIMELIGHT

            LLResult result = limelight.getLatestResult();

//            telemetry.addData("Valid: ", result.isValid());
            if(gamepad1.dpad_up ){
                elevatorPos+=10;
                extendTo(elevatorPos,0.5);
            }

            if(gamepad1.right_trigger>0.5){

                //Field pos storing
                try {
//                telemetry.addData("result", result.getDetectorResults().get(0).getTargetCorners());


                    List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
                    List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
                    List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
                    List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);

                    double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                    double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;

                    Point centroid = new Point(cx, cy);

                    Point obj = get_field_pos(centroid);

                    telemetry.addData("centroid", new Point(cx, cy));

                    telemetry.addData("Field_Pos", obj);



                    if(obj.x >0) {
                        prev_obj_pos = obj_pos;
                        obj_pos = Math.min(obj.x*k + prev_obj_pos*(1-k),40);
                    }
                    extendTo(offsetDist+InchesToEncoderTicks(obj_pos),1);
                }

                catch (Exception e){
                    telemetry.addLine("No point found");
                }

            }
            else if (gamepad1.left_trigger>0.5){
                extendToInches(0,1);
            }

            if(elevatorPos>=1300){
                elevatorPos=1300;
            }
            if(elevatorPos<=0){
                elevatorPos=0;
            }

            if (gamepad1.a) {
                stateBased(RobotState.PICK);
            } else if (gamepad1.right_bumper) {
                stateBased(RobotState.FOLLOWER);
            } else if (gamepad1.left_bumper  ) {
                stateBased(RobotState.DROP);
            }

            if(gamepad1.b){
                stateBased(RobotState.INIT);
            }


            if(gamepad1.x){
                gripper(GripperState.OPEN);
            }


            //TODO WRIST ROTATED
            boolean currentButtonState = gamepad1.y;
            if (currentButtonState && !previousButtonState) {
                toggle = !toggle;
            }
            previousButtonState = currentButtonState;

            if (toggle) {
                wrist(WristState.VERT);
            } else {
                wrist(WristState.HORIZONTAL);
            }

            if (shoulderState == ShoulderState.FOLLOWER) {
                // Follower logic to update continuously
                int currentEncoderValue = robot.lifterL.getCurrentPosition();//
                double servoPosition = minServoPosition + ((double)(currentEncoderValue - minEncoderValue) / (maxEncoderValue - minEncoderValue)) * (maxServoPosition - minServoPosition);
                robot.shoulder.setPosition(servoPosition);
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)),-gamepad1.right_stick_x));
            drive.updatePoseEstimate();


            printStuff();
            telemetry.update();
        }
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

    public void stateBased(RobotState state){
        robotState=state;
        switch (state){
            case PICK:
                //TODO ADD SEQ
                shoulder(ShoulderState.PICK);
                sleep(500); // Wait for half a second
                gripper(GripperState.CLOSE);
                sleep(250);
                shoulder(ShoulderState.PREPICK);
//                currentServoState = ServoState.FOLLOW_SLIDER;
                break;
            case DROP:
                //TODO ADD SEQ
                DROP();
                break;
            case INIT:  //IDLE STATE
                //TODO ADD SEQ
                INIT();
                break;
            case FOLLOWER:
                //follower logic
                //TODO ADD SEQ
                shoulder(ShoulderState.FOLLOWER);
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

    public void INIT(){
        extendTo(0,1);
        extendTurret(0,0.4);
        sleep(300);
        wrist(WristState.HORIZONTAL);
        viper(ViperState.INIT);
        shoulder(ShoulderState.INIT);
        sleep(250);
        extendTurret(0,0.5);
    }

    public void DROP(){
        wrist(WristState.HORIZONTAL);
        viper(ViperState.DROP);
        shoulder(ShoulderState.INIT);
        sleep(250);
        extendTurret(1296,1);
        sleep(500);
        extendTo(2900,1);
    }

    public  void reset(){
        robot.lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftChanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lowHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RUN_USING_ENC(){
        robot.lifterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lifterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftChanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lowHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER); }

    public void extendTo(int Target,double pow){
        robot.lifterL.setTargetPosition(Target);
        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterL.setPower(pow);
        robot.lifterR.setTargetPosition(Target);
        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterR.setPower(pow);
    }

    public void extendToInches(int Target,double pow){
        robot.lifterL.setTargetPosition((int)encoderTicksToInches(Target));
        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterL.setPower(pow);
        robot.lifterR.setTargetPosition((int)encoderTicksToInches(Target));
        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterR.setPower(pow);
    }

    public static double encoderTicksToInches(double ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static int InchesToEncoderTicks(double inches) {
        return (int)(1/(PULLEY_RADIUS * 2 * Math.PI * GEAR_RATIO / TICKS_PER_REV) * inches/2.2);
    }

    private Point get_field_pos(Point centroid) {
        double theta = Math.toRadians(15);

        double x0 = 480 - 480;

        double x1 = 480 - centroid.y;
        double y = 320 - centroid.x;

        double x2 = 480 - 240;
        double x3 = 480 - 169;
        double f0 = 0;
        double f2 = 8.25;
        double f3 = 8.25+9.35;

        double k = (x2-x0)*(x3-x1)*(f3-f0)/(x3-x0)/(x2-x1)/(f2-f0);

        double cam_x = (k*f2 - f3)/(k-1);
        double cam_y = 0;

//        double field_x = cam_x*Math.cos(theta) + cam_y*Math.sin(theta);
//        double field_y = cam_y*Math.cos(theta) - cam_x*Math.sin(theta) + 5.5;

//        return new Point(field_x,field_y);
        return new Point(cam_x+6.75,cam_y);
    }

    public void extendTurret(int Target,double pow){
        robot.liftChanger.setTargetPosition(Target);
        robot.liftChanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftChanger.setPower(pow);
    }

    public void extendLowHang(int Target,double pow){
        robot.lowHang.setTargetPosition(Target);
        robot.lowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lowHang.setPower(pow);
    }

    //TODO STATES
    enum ShoulderState{
        PICK,
        PREPICK,
        DROP,
        INIT,
        FOLLOWER,
        NOTFOLLOWER,
    }
    enum GripperState{
        OPEN,
        CLOSE,
    }
    enum WristState {
        HORIZONTAL,
        VERT,
    }
    enum ViperState {
        INIT,
        PICK,
        DROP,
    }
    enum RobotState {
        INIT,
        PICK,
        DROP,
        FOLLOWER,
        GRIPPER,
    }
}
