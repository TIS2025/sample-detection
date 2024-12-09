package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.opengl.models.SavedMeshObject;
import org.firstinspires.ftc.teamcode.LL.Sample;
import org.firstinspires.ftc.teamcode.VisionUtils.CameraOrientation;
import org.firstinspires.ftc.teamcode.VisionUtils.KinematicSolver;
import org.firstinspires.ftc.teamcode.VisionUtils.PerspectiveSolver;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

//viperneutral - 0.175
//viperright90 - 0.51
//wrist 90 - 0.35
//wrist 0 - 0.85
//
//ext - 0-2500 => 0-23
//angle - 17
//shoulder min = 0.36
//shoulder max = 0.5
//shoulder 0 - 0.44
//shoulder +90 - 0.77
//
//l_arm = 6
//
//tur angle 1241/1160
//lifter ext - 200

@TeleOp(name="Yash Obj pick")
@Config
public class ObjPick extends LinearOpMode {

    public static Limelight3A limelight;

    int CAMERA_HEIGHT = 480;
    int CAMERA_WIDTH = 640;

    double w1 = 7.3;
    double w2 = 16.5;
    double cx1 = CAMERA_HEIGHT - 480;
    double cx2 = CAMERA_HEIGHT - 164;
    double cx3 = CAMERA_HEIGHT - 73;

    public List<Sample> redSamples = new ArrayList<>();
    public List<Sample> blueSamples = new ArrayList<>();
    public List<Sample> yellowSamples = new ArrayList<>();
//    Sample x = new Sample();

    public static double angle = 28.5;
    double cam_offset = 7;
    double x_offset = 0;
    double y_offset = 4.9;
    Point obj_pos = new Point(0,0);
    Point[] prev_pos = {new Point(0,0),new Point(0,0),new Point(0,0)};
    boolean obj_orient = false;

    PerspectiveSolver Psolver = new PerspectiveSolver(angle,x_offset,y_offset,cam_offset,cx1,cx2,cx3,0,10,20,
            0,0,0,0,0,0,w1,w2, CameraOrientation.UPRIGHT,CAMERA_HEIGHT,CAMERA_WIDTH);

    KinematicSolver solver = new KinematicSolver(1.5,5.2,0.5,0);


    Servo shoulder;
    Servo wrist;
    Servo grip;
    Servo viper;

    Servo s1,s2,s3,s4,s5,s6;

    double[] pickup_pos = new double[0];
    public static double tx = 10,ty=5;

    //TODO SERVO VAR

    public static double shoulderL=0.5;
    public static double wristPos=0.5;
    public static double viperPos=0.5;
    public static double gripPos=0.5;

    //TODO MOTOR
    DcMotorEx lifterL=null;
    DcMotorEx lifterR=null;
    DcMotorEx liftChanger=null;
    DcMotorEx lowHang=null;

    //TODO MOTOR-VAR
    public static int elevatorPos=0;
    public static int liftChangerPos=0;
    public static int lowHangPos=0;
    public static double pow=0.8;
    public double inchExt = 0;
    double shPos;

    boolean track_flag = false;
    boolean update_reading = false;



    @Override
    public void runOpMode() throws InterruptedException {
        shoulder=hardwareMap.get(Servo.class,"ls");
        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist=hardwareMap.get(Servo.class,"wt");
        grip=hardwareMap.get(Servo.class,"gp");
        viper=hardwareMap.get(Servo.class,"vp");
        viper.setDirection(Servo.Direction.REVERSE);

        lifterL=hardwareMap.get(DcMotorEx.class,"ll");
        lifterR=hardwareMap.get(DcMotorEx.class,"lr");
        liftChanger=hardwareMap.get(DcMotorEx.class,"lc");
        lowHang=hardwareMap.get(DcMotorEx.class,"lHang");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(2);

        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();
        double[] pos = new double[2];


//        s1=hardwareMap.get(Servo.class,"s1");
//        s2=hardwareMap.get(Servo.class,"s2");
//        s3=hardwareMap.get(Servo.class,"s3");
//        s4=hardwareMap.get(Servo.class,"s4");
//        s5=hardwareMap.get(Servo.class,"s5");
//        s6=hardwareMap.get(Servo.class,"s6");


        lifterL.setDirection(DcMotorSimple.Direction.REVERSE);

        RUN_USING_ENC();

        wristPos=0.5;
        gripPos=0.9;
        shoulderL=0.67;
        viperPos=0.82;
        viper.setPosition(viperPos);
        shoulder.setPosition(shoulderL);
        wrist.setPosition(wristPos);
        grip.setPosition(gripPos);


        while (opModeInInit()){
            if(gamepad1.dpad_up){
                elevatorPos+=10;
                extendToNoClip(elevatorPos,pow);
            } else if (gamepad1.dpad_down) {
                elevatorPos-=10;
                extendToNoClip(elevatorPos,pow);
            } //+up  =down

            if(gamepad1.dpad_left){
                liftChangerPos +=10;
                extendTurret(liftChangerPos,pow);
            } else if (gamepad1.dpad_right) {
                liftChangerPos -=10;
                extendTurret(liftChangerPos,pow);
            } //- pick +drop

            if(gamepad1.left_bumper){
                lowHangPos+=10;
                extendLowHang(lowHangPos,pow);
            } else if (gamepad1.right_bumper) {
                lowHangPos-=10;
                extendLowHang(lowHangPos,pow);
            }

            if(gamepad1.start){
                s1.setPosition(0.5);
                s2.setPosition(0.5);
                s3.setPosition(0.5);
                s4.setPosition(0.5);
                s5.setPosition(0.5);
                s6.setPosition(0.5);
            } else if (gamepad1.y) {
                s1.setPosition(0);
                s2.setPosition(0);
                s3.setPosition(0);
                s4.setPosition(0);
                s5.setPosition(0);
                s6.setPosition(0);
            } else if (gamepad1.x) {
                s1.setPosition(1);
                s2.setPosition(1);
                s3.setPosition(1);
                s4.setPosition(1);
                s5.setPosition(1);
                s6.setPosition(1);
            }

            telemetry.addData("LIFTER L",lifterL.getCurrentPosition());
            telemetry.addData("LIFTER R",lifterR.getCurrentPosition());
            telemetry.addData("TURRET R",liftChanger.getCurrentPosition());
            telemetry.addData("LOW HANG",lowHang.getCurrentPosition());

//            if(!limelight.isConnected()){
////                limelight.setPollRateHz(100);
//                limelight.stop();
//                sleep(1000);
//                limelight.start();
//            }


            telemetry.addData("Connected",limelight.isConnected());
            telemetry.addData("Connected",limelight.isRunning());

            telemetry.update();
        }

        reset();
        waitForStart();
        limelight.start();
        while (opModeIsActive()) {
            P.copy(C);
            C.copy(gamepad1);
            redSamples.clear();
            blueSamples.clear();
            yellowSamples.clear();

            LLResult result = limelight.getLatestResult();
            sample_filter(result);
            if(!redSamples.isEmpty()) {
                telemetry.addData("Red Samples",redSamples.size());
                telemetry.addData("Sample",redSamples.get(0).field_pos);
            }
            else telemetry.addData("Red Samples",0);
            if(!blueSamples.isEmpty())  {
                telemetry.addData("Blue Samples",blueSamples.size());
                telemetry.addData("Sample",blueSamples.get(0).field_pos);
            }
            else telemetry.addData("Blue Samples",0);
            if(!yellowSamples.isEmpty())  {
                telemetry.addData("Yellow Samples",yellowSamples.size());
                telemetry.addData("Sample",yellowSamples.get(0).field_pos);
            }
            else telemetry.addData("Yellow Samples",0);


//            if(C.a && !P.a) track_flag = !track_flag;
//            if(C.x && !P.x) update_reading = !update_reading;

//            if(update_reading){
////                interpret_limelight(result);
//
//                sample_filter(result);
//
////                telemetry.addData("Blue Samples",blueSamples.size());
////                telemetry.addData("Yellow Samples",yellowSamples.size());
////                pos = solver.getExtYaw(obj_pos);
//            }

            if(C.y && !P.y && !redSamples.isEmpty()){
                pickup_pos = solver.getExtYaw(redSamples.get(0).field_pos);
                obj_orient = redSamples.get(0).orientation;
                {
                    extendToInchInput(pickup_pos[0],0.9);
                    viperYawDegrees(pickup_pos[1]);
                    wristRotateOrientation(pickup_pos[1],obj_orient);
                    sleep(500);
                    shoulder.setPosition(0.65);
                    sleep(500);
                    grip.setPosition(0.938);
                    sleep(500);
                    shoulder.setPosition(0.8);
                }
            }
            if(C.x && !P.x && !yellowSamples.isEmpty()){
                pickup_pos = solver.getExtYaw(yellowSamples.get(0).field_pos);
                obj_orient = yellowSamples.get(0).orientation;
                {
                    extendToInchInput(pickup_pos[0],0.9);
                    viperYawDegrees(pickup_pos[1]);
                    wristRotateOrientation(pickup_pos[1],obj_orient);
                    sleep(500);
                    shoulder.setPosition(0.65);
                    sleep(500);
                    grip.setPosition(0.938);
                    sleep(500);
                    shoulder.setPosition(0.8);
                }
            }
            if(C.a && !P.a && !blueSamples.isEmpty()){
                pickup_pos = solver.getExtYaw(blueSamples.get(0).field_pos);
                obj_orient = blueSamples.get(0).orientation;
                {
                    extendToInchInput(pickup_pos[0],0.9);
                    viperYawDegrees(pickup_pos[1]);
                    wristRotateOrientation(pickup_pos[1],obj_orient);
                    sleep(500);
                    shoulder.setPosition(0.65);
                    sleep(500);
                    grip.setPosition(0.938);
                    sleep(500);
                    shoulder.setPosition(0.8);
                }
            }

//            if (track_flag){
//                update_reading = false;
//                interpret_limelight(result);
//                pos = solver.getExtYaw(obj_pos);
//                if(obj_pos.x>=3 && Math.abs(obj_pos.y)<6)
//                {
//                    extendToInchInput(pos[0],pow);
//                    viperYawDegrees(pos[1]);
//                }
//            }

            //TODO SHOULDER-POS
//            if(C.dpad_up && !P.dpad_up){
//                shoulderL+=0.001;
//                shoulder.setPosition(shoulder.getPosition()+0.01);
//            } else if (C.dpad_down && !P.dpad_down) {
//                shoulderL-=0.001;
//                shoulder.setPosition(shoulder.getPosition()-0.01);
//            }
//
            if(gamepad1.dpad_left){
                viperPos+=0.001;
                viper.setPosition(viperPos);
            } else if (gamepad1.dpad_right) {
                viperPos-=0.001;
                viper.setPosition(viperPos);
            }

            //viper3 ---gripper ac
            //shoulder0-- wrist
            //wrist1 -viper

            //TODO WRIST-POS
            if(C.dpad_up){
                wristPos+=0.001;
                wrist.setPosition(wristPos);
            } else if (C.dpad_down) {
                wristPos-=0.001;
                wrist.setPosition(wristPos);
            }

            //TODO GRIPPER-POS
//            if(C.right_bumper && !P.right_bumper){
//                inchExt+=1;
//                inchExt = Math.min(inchExt,11);
//                extendToInchInputWithShoulder(inchExt,pow);
//            } else if (C.left_bumper && !P.left_bumper) {
//                inchExt-=1;
//                inchExt = Math.max(inchExt,0);
//                extendToInchInputWithShoulder(inchExt,pow);
//            }

//            if(C.a){
//                interpret_limelight(result);
//                pos = solver.getExtYaw(obj_pos);
//            }
//            if(C.x){
//                wrist.setPosition(obj_orient?0.35:0.85);
//                extendToInchInputWithShoulder(pos,pow);
//                viperYawDegrees(pos[1]);
//            }
//            if(C.y){
//                shoulder.setPosition(shPos);
//                sleep(300);
//                grip.setPosition(0.938);
//            }
            if(C.b){
                viperYawDegrees(0);
                shoulder.setPosition(0.78);
//                sleep(500);
                extendTo(5,pow);
                wrist.setPosition(0.85);
                grip.setPosition(0.75);
                track_flag=false;
            }

            if(shoulderL<0){
                shoulderL=0;
            }
            if(shoulderL>1){
                shoulderL=1;
            }


            if(wristPos<0){
                wristPos=0;
            }
            if(wristPos>1){
                wristPos=1;
            }
            if(gripPos<0){
                gripPos=0;
            }
            if(gripPos>1){
                gripPos=1;
            }
            if(viperPos<0){
                viperPos=0;
            }
            if(viperPos>1){
                viperPos=1;
            }

            //gripperf clo -0.938 open 0.39

            if(gamepad1.left_trigger>0.3){
                elevatorPos+=10;
//                elevatorPos = Math.max(0, Math.min(1200, elevatorPos));
//                extendTo(elevatorPos,pow);
            } else if (gamepad1.right_trigger>0.3) {
                elevatorPos-=10;
//                elevatorPos = Math.max(0, Math.min(1200, elevatorPos));
//                extendTo(elevatorPos,pow);
            } //+up  =down
//            extendTo(elevatorPos,pow);
            if(gamepad1.back){
                liftChangerPos +=10;
                liftChangerPos = Math.max(liftChangerPos,0);
                extendTurret(liftChangerPos,pow);
            } else if (gamepad1.start) {
                liftChangerPos -=10;
                liftChangerPos = Math.max(liftChangerPos,0);
                extendTurret(liftChangerPos,pow);
            } //- pic

            String field_pos0 = "{" + String.format("%.2f",prev_pos[2].x) + "," + String.format("%.2f",prev_pos[2].y) + "}";
            String field_pos_1 = "{" + String.format("%.2f",prev_pos[1].x) + "," + String.format("%.2f",prev_pos[1].y) + "}";
            String field_pos_2 = "{" + String.format("%.2f",prev_pos[0].x) + "," + String.format("%.2f",prev_pos[0].y) + "}";

//            telemetry.addData("field_pos cur",field_pos0);
//            telemetry.addData("field_pos prev",field_pos_1);
//            telemetry.addData("field_pos prev prev",field_pos_2);
            telemetry.addData("Update",update_reading);
//            telemetry.addData("track",track_flag);
//            telemetry.addData("wrist_rotate",obj_orient);
//            telemetry.addData("LIFTER L",lifterL.getCurrentPosition());
//            telemetry.addData("LIFTER R",lifterR.getCurrentPosition());
//            telemetry.addData("Inch ext",inchExt);
//            telemetry.addData("TURRET R",liftChanger.getCurrentPosition());
//            telemetry.addData("LOW HANG",lowHang.getCurrentPosition());
//
//
//            //TODO TELEMETRY
//            telemetry.addData("Shoulder L", shoulderL);
//            telemetry.addData("Shoulder LEFT", shoulder.getPosition());
//            telemetry.addData("Wrist", wristPos);
//            telemetry.addData("Wrist SERVO", wrist.getPosition());
//            telemetry.addData("Gripper", gripPos);
//            telemetry.addData("Viper SERVO",viper.getPosition());
//            telemetry.addData("Viper",viperPos);
//
//            telemetry.addData("Kin ext","%.2f",pos[0]);
//            telemetry.addData("Kin yaw","%.2f",pos[1]);
            telemetry.update();
        }


    }

    private Point average_reading() {
        double x = (prev_pos[0].x+prev_pos[1].x+prev_pos[2].x)/3;
        double y = (prev_pos[0].y+prev_pos[1].y+prev_pos[2].y)/3;
        return new Point(x,y);
    }

    private void viperYawDegrees(double degrees){
        viper.setPosition(0.82-(degrees/270));
    }

    private void wristRotateOrientation(double degrees,boolean rotate){
        double wristYaw = rotate?0.17:0.67;
        wristYaw+=(degrees/180);
        if(wristYaw<0) wristYaw = 1+wristYaw;
        if(wristYaw>1) wristYaw = 1-wristYaw;
        wrist.setPosition(wristYaw);
    }

    private void extendToInchInput(double InchExt, double pow) {
        double inch = Math.max(0,Math.min(InchExt,11));
        double tickPerInch = 2500/23.0;
        int inp = (int)(inch*tickPerInch);
        extendTo(inp,pow);
//        sleep(750);
//        shoulder.setPosition(shPos);
    }

    private void extendToNoClip(int Target, double pow) {
        lifterL.setTargetPosition(Target);
        lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifterL.setPower(pow);

        lifterR.setTargetPosition(Target);
        lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifterR.setPower(pow);
    }


    public void extendTo(int Target,double pow){
        int inp = Math.max(0, Math.min(1200, Target));
        lifterL.setTargetPosition(inp);
        lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifterL.setPower(pow);

        lifterR.setTargetPosition(inp);
        lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifterR.setPower(pow);
    }

    public void extendTurret(int Target,double pow){
        liftChanger.setTargetPosition(Target);
        liftChanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftChanger.setPower(pow);
    }

    public void extendLowHang(int Target,double pow){
        lowHang.setTargetPosition(Target);
        lowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowHang.setPower(pow);
    }

    public  void reset(){
        lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftChanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RUN_USING_ENC(){
        lifterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftChanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void interpret_limelight(LLResult result){

        try{
            telemetry.addData("No of samples detected",result.getDetectorResults().size());
            List<Double> pt1 = result.getDetectorResults().get(0).getTargetCorners().get(0);
            List<Double> pt2 = result.getDetectorResults().get(0).getTargetCorners().get(1);
            List<Double> pt3 = result.getDetectorResults().get(0).getTargetCorners().get(2);
            List<Double> pt4 = result.getDetectorResults().get(0).getTargetCorners().get(3);

            double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
            double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
            double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
            double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

            double width = max_x-min_x;
            double height = max_y-min_y;

            double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
            double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
//            double cy = (pt1.get(1) + pt2.get(1) + pt3.get(1) + pt4.get(1)) / 4;

            Point centroid = new Point(cx, cy);
            obj_pos = Psolver.getX2Y2(centroid);
            obj_orient = width/height>1.2;
            telemetry.addData("Width/Height","%.3f",width/height);
            double offset = obj_orient? 0.5:1.5;
            obj_pos.x+=offset;
            prev_pos[0] = prev_pos[1];
            prev_pos[1] = prev_pos[2];
            prev_pos[2] = obj_pos;
        }
        catch (Exception e){
            obj_pos = new Point(0,0);
        }
    }

    public void sample_filter(LLResult result){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                List<Double> pt1 = detector.getTargetCorners().get(0);
                List<Double> pt2 = detector.getTargetCorners().get(1);
                List<Double> pt3 = detector.getTargetCorners().get(2);
                List<Double> pt4 = detector.getTargetCorners().get(3);

                double max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                double min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                double max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                double min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));

                double width = max_x - min_x;
                double height = max_y - min_y;

                double cx = (pt1.get(0) + pt2.get(0) + pt3.get(0) + pt4.get(0)) / 4;
                double cy = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));

                Point field_pos = Psolver.getX2Y2(new Point(cx, cy));
                boolean orientation = width/height>1.2;
                int class_id = detector.getClassId();
                double confidence = detector.getConfidence();
                double offset = orientation?0.5:1.5;

                if (Math.abs(field_pos.y+1.5) < 1.5 && field_pos.x < 15 && class_id == 0) {
                    field_pos.x+=offset;
                    blueSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y+1.5) < 1.5 && field_pos.x < 15 && class_id == 1) {
                    field_pos.x+=offset;
                    redSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
                if (Math.abs(field_pos.y+1.5) < 1.5 && field_pos.x < 15 && class_id == 2) {
                    field_pos.x+=offset;
                    yellowSamples.add(new Sample(field_pos,class_id,confidence,orientation));
                }
            }
        }
        catch (Exception e){
            telemetry.addLine("Failed to input samples");
        }
    }

}

/*
PICK
lifter-577
lift changer- -1117


//INIT
shoulder -1
wrist-  -0.8089
viper-0.338

//drop
lifter-2853
lift changer- 0
shoulder -0.0678
wrist-  -0.8089
viper-1



 */



//TODO PICK PVALUES
/*
shoulL=0;
wrist=0.83
grip -close-0.559  open -0.431
viper- 0.344



 */

//TODO TRANSFER

/*

//1
shoulL=0.3989;
wrist=0.83
grip -close-0.559  open -0.431
viper- 0.647

//2
shoulL=0.85;
wrist=0.83
grip -close-0.559  open -0.431
viper- 0.647

 */