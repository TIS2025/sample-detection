package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color")
@Config
public class ColorTest extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {

         dc = hardwareMap.get(DcMotorEx.class, "dc");

        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");


        NormalizedRGBA rgba;
        float[] hsv;
        double distance;
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(0.5);
        colorSensor.setGain((float)gain);

        Gamepad C = new Gamepad();
        Gamepad P = new Gamepad();

        waitForStart();
        while (opModeIsActive()) {

            P.copy(C);
            C.copy(gamepad1);

            rgba = colorSensor.getNormalizedColors();
            hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);
            distance = colorSensor.getDistance(DistanceUnit.MM);

            //Motor Test
            if(gamepad1.b){dc.setPower(dcval);}
            if(dc.getCurrent(CurrentUnit.MILLIAMPS)>amp){dc.setPower(0);}
            telemetry.addData("Current: ",dc.getCurrent(CurrentUnit.MILLIAMPS));

            //Color test conditions
            if((hsv[0]<redHigh && hsv[0]>redLow && distance>distLimit && hsv[1]<satLimit)||
               (hsv[0]<blueHigh && hsv[0]>blueLow && distance>distLimit && hsv[1]<satLimit))
                {gripper.setPosition(0.25);}

            if(gamepad1.a){gripper.setPosition(0.5);}

            telemetry.addLine()
                    .addData("Red", rgba.red)
                    .addData("Green", rgba.green)
                    .addData("Blue", rgba.blue);
            telemetry.addLine()
                    .addData("Hue", hsv[0])
                    .addData("Saturation", hsv[1])
                    .addData("Value", hsv[2]);
            telemetry.addLine().addData("Distance",distance);

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
