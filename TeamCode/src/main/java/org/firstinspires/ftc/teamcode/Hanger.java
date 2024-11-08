package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="Hanger TeleOp")
@Config
public class Hanger extends LinearOpMode {
    Servo shoulder;
    Servo wrist;
    Servo grip;
    Servo viper;

    Servo s1,s2,s3,s4,s5,s6;



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
    public static double pow=0.3;
    public double inchExt = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        shoulder=hardwareMap.get(Servo.class,"ls");
        wrist=hardwareMap.get(Servo.class,"wt");
        grip=hardwareMap.get(Servo.class,"gp");
        viper=hardwareMap.get(Servo.class,"vp");

        lifterL=hardwareMap.get(DcMotorEx.class,"ll");
        lifterR=hardwareMap.get(DcMotorEx.class,"lr");
        liftChanger=hardwareMap.get(DcMotorEx.class,"lc");
        lowHang=hardwareMap.get(DcMotorEx.class,"lHang");
        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();

//        s1=hardwareMap.get(Servo.class,"s1");
//        s2=hardwareMap.get(Servo.class,"s2");
//        s3=hardwareMap.get(Servo.class,"s3");
//        s4=hardwareMap.get(Servo.class,"s4");
//        s5=hardwareMap.get(Servo.class,"s5");
//        s6=hardwareMap.get(Servo.class,"s6");


        lifterL.setDirection(DcMotorSimple.Direction.REVERSE);

        RUN_USING_ENC();

        wristPos=0.5;
        gripPos=0.5;
        shoulderL=0.36;
        viperPos=0.19;
        viper.setPosition(viperPos);
        shoulder.setPosition(shoulderL);
        wrist.setPosition(wristPos);
        grip.setPosition(gripPos);

        ElapsedTime timer = new ElapsedTime();


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
            telemetry.update();
        }
        reset();
        waitForStart();
        while (opModeIsActive()) {
            P.copy(C);
            C.copy(gamepad1);



            //TODO SHOULDER-POS
            if(gamepad1.dpad_up){
                shoulderL+=0.001;
                shoulder.setPosition(shoulderL);
            } else if (gamepad1.dpad_down) {
                shoulderL-=0.001;
                shoulder.setPosition(shoulderL);
            }
//
            if(gamepad1.dpad_left){
                viperPos+=0.001;
                viper.setPosition(viperPos);
            } else if (gamepad1.dpad_right) {
                viperPos-=0.001;
                viper.setPosition(viperPos);

            }

            if(gamepad1.a){
                extendHanger(660,0.8);
                extendTurret(1100,0.8);
                extendToNoClip(180,0.8);
            }

            if(gamepad1.b){
                extendHanger(225,0.8);
                extendToNoClip(2900,0.8);
            }

            if(gamepad1.x){
                extendTurret(1270,0.8);
//                sleep(1000);
            }

            if(gamepad1.y){
                extendToNoClip(600,0.8);
            }

            if(gamepad1.left_bumper){
                extendHanger(0,0.8);
                extendTurret(0,0.8);
                extendToNoClip(0,0.8);
            }


//
//            //viper3 ---gripper ac
//            //shoulder0-- wrist
//            //wrist1 -viper
//
//            //TODO WRIST-POS
//            if(gamepad1.a){
//                wristPos+=0.001;
//                wrist.setPosition(wristPos);
//            } else if (gamepad1.b) {
//                wristPos-=0.001;
//                wrist.setPosition(wristPos);
//            }
//
//            //TODO GRIPPER-POS
//            if(C.right_bumper && !P.right_bumper){
//                inchExt+=1;
//                inchExt = Math.min(inchExt,11);
//                extendToInchInputWithShoulder(inchExt,pow,17);
//            } else if (C.left_bumper && !P.left_bumper) {
//                inchExt-=1;
//                inchExt = Math.max(inchExt,0);
//                extendToInchInputWithShoulder(inchExt,pow,17);
//            }
//
//
//            if(shoulderL<0){
//                shoulderL=0;
//            }
//            if(shoulderL>1){
//                shoulderL=1;
//            }
//
//
//            if(wristPos<0){
//                wristPos=0;
//            }
//            if(wristPos>1){
//                wristPos=1;
//            }
//            if(gripPos<0){
//                gripPos=0;
//            }
//            if(gripPos>1){
//                gripPos=1;
//            }
//            if(viperPos<0){
//                viperPos=0;
//            }
//            if(viperPos>1){
//                viperPos=1;
//            }
//
//            //gripperf clo -0.938 open 0.39
//
//
//
//
//            if(gamepad1.left_trigger>0.3){
//                elevatorPos+=10;
////                elevatorPos = Math.max(0, Math.min(1200, elevatorPos));
////                extendTo(elevatorPos,pow);
//            } else if (gamepad1.right_trigger>0.3) {
//                elevatorPos-=10;
////                elevatorPos = Math.max(0, Math.min(1200, elevatorPos));
////                extendTo(elevatorPos,pow);
//            } //+up  =down
////            extendTo(elevatorPos,pow);
//            if(gamepad1.back){
//                liftChangerPos +=10;
//                liftChangerPos = Math.max(liftChangerPos,0);
//                extendTurret(liftChangerPos,pow);
//            } else if (gamepad1.start) {
//                liftChangerPos -=10;
//                liftChangerPos = Math.max(liftChangerPos,0);
//                extendTurret(liftChangerPos,pow);
//            } //- pic
//
//
//            if(gamepad2.a){
//                //TODO PICK
//
//            }

            telemetry.addData("LIFTER L",lifterL.getCurrentPosition());
            telemetry.addData("LIFTER R",lifterR.getCurrentPosition());
            telemetry.addData("Inch ext",inchExt);
            telemetry.addData("TURRET R",liftChanger.getCurrentPosition());
            telemetry.addData("LOW HANG",lowHang.getCurrentPosition());


            //TODO TELEMETRY
            telemetry.addData("Shoulder L", shoulderL);
            telemetry.addData("Shoulder LEFT", shoulder.getPosition());
            telemetry.addData("Wrist", wristPos);
            telemetry.addData("Wrist SERVO", wrist.getPosition());
            telemetry.addData("Gripper", gripPos);
            telemetry.addData("Viper SERVO",viper.getPosition());
            telemetry.addData("Viper",viperPos);



            telemetry.update();
        }


    }

    private void extendToInchInputWithShoulder(double inchExt, double pow, double angle) {
        double inch = Math.max(0,Math.min(inchExt,11));
        double tickPerInch = 2500/23.0;
        int inp = (int)(inch/Math.cos(Math.toRadians(angle))*tickPerInch);
        extendTo(inp,pow);

        double shPos = 0.36 + (0.5-0.36)*inch/11.0;
        shoulder.setPosition(shPos);
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

    public void extendHanger(int Target,double pow){
        lowHang.setTargetPosition(Target);
        lowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowHang.setPower(pow);
    }

    public void extendToInchInput(double Target,double pow,double angle){
        double tickPerInch = 2500/23.0;
        int inp = (int)(Target/Math.cos(Math.toRadians(angle))*tickPerInch);
        extendTo(inp,pow);
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