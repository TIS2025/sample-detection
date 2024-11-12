package org.firstinspires.ftc.teamcode.LL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Test")
@Disabled
public class Limelight_basic extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        telemetry.setMsTransmissionInterval(100);
        limelight.pipelineSwitch(2);

        waitForStart();
        limelight.start();
        while (opModeIsActive()){


            LLResult result = limelight.getLatestResult();
//            telemetry.addData("Result null",result==null);
//            if (result != null) {
//                telemetry.addData("Validity",result.isValid());
//                if (result.isValid()) {
//                    double[] pythonOutputs = result.getPythonOutput();
//                    telemetry.addData("Validity", result.isValid());
//                    telemetry.addLine();
//                    telemetry.addLine();
                    telemetry.addData("Result", result);
//                }
//            }


//            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//            if(result.isValid()){
//            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//                telemetry.addData("Out", )
//            }
            telemetry.update();
        }

//        limelight.stop();
    }
}
