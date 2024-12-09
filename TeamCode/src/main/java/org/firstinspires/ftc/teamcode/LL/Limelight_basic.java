package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Basic Test")
public class Limelight_basic extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(0);

        waitForStart();
        limelight.start();
        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
                    telemetry.addData("Result", result);
            telemetry.update();
        }

//        limelight.stop();
    }
}
