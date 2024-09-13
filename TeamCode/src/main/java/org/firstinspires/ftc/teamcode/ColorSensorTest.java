package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color")
@Config
public class ColorSensorTest extends LinearOpMode {

    public static DcMotorEx dc;
    public static double dcval = 0.8;
    public static double gain = 50;
    public static double amp = 2000;

    public static double redHigh = 30;
    public static double redLow = 15;
    public static double blueHigh = 220;
    public static double blueLow = 200;
    public static double yellowLow = 80;
    public static double yellowHigh = 95;
    public static double satLimit = 0.8;
    public static double distLimit = 10;

    public static int lift_height=0;

    @Override
    public void runOpMode() throws InterruptedException {

//         dc = hardwareMap.get(DcMotorEx.class, "dc");

        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");


        NormalizedRGBA rgba;
        float[] hsv;
        double distance;
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(0.4);
        colorSensor.setGain((float)gain);

        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0.55);

        Servo elbow = hardwareMap.get(Servo.class, "elbow");
        elbow.setPosition(0.5);

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class,"motor1");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

        Trajectory pickupSample = TrajectoryActionBuilder(
                new SequentialAction(()->)
        )

        waitForStart();
        while (opModeIsActive()) {

            P1.copy(C1);
            C1.copy(gamepad1);

            P2.copy(C2);
            C2.copy(gamepad2);

            if(C1.dpad_up && !P1.dpad_up) {
                lift_height+=10;
                lift.setTargetPosition(lift_height);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.8);
            }

            if(C1.dpad_down && !P1.dpad_down) {
                lift_height-=10;
                lift.setTargetPosition(lift_height);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(0.8);
            }

            if(C1.left_bumper && !P1.left_bumper) elbow.setPosition(elbow.getPosition()+0.05);
            if(C1.right_bumper && !P1.right_bumper) elbow.setPosition(elbow.getPosition()-0.05);

            rgba = colorSensor.getNormalizedColors();
            hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);
            distance = colorSensor.getDistance(DistanceUnit.MM);

            //Motor Test
//            if(gamepad1.b){dc.setPower(dcval);}
//            if(dc.getCurrent(CurrentUnit.MILLIAMPS)>amp){dc.setPower(0);}
//            telemetry.addData("Current: ",dc.getCurrent(CurrentUnit.MILLIAMPS));

            //Color test conditions
            if((hsv[0]<redHigh && hsv[0]>redLow && hsv[1]<satLimit)||
               (hsv[0]<blueHigh && hsv[0]>blueLow && hsv[1]<satLimit))
                {gripper.setPosition(0.15);}

            if(C1.a){
                gripper.setPosition(0.4);
            }

            telemetry.addLine()
                    .addData("Red", rgba.red)
                    .addData("Green", rgba.green)
                    .addData("Blue", rgba.blue);
            telemetry.addLine()
                    .addData("Hue", hsv[0])
                    .addData("Saturation", hsv[1])
                    .addData("Value", hsv[2]);
            telemetry.addLine().addData("Distance",distance);
            telemetry.addLine().addData("Lift height:",lift.getCurrentPosition());

            telemetry.update();
        }
    }



    private float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;

        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }

        return hsv;
    }
}
