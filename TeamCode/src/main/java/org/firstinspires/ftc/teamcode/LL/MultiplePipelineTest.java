package org.firstinspires.ftc.teamcode.LL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "Multiple Pipeline")
public class MultiplePipelineTest extends LinearOpMode {
    Limelight3A limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
    public static final FtcDashboard dash = FtcDashboard.getInstance();

    double min_x,max_x,min_y,max_y;
    int class_id;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");

        int pipeline = 0;
        limelight3A.pipelineSwitch(pipeline);
        limelight3A.updatePythonInputs(0,0,640,480,0,0,0,0);


        waitForStart();
        limelight3A.start();
        while (opModeIsActive()){
            LLResult result = limelight3A.getLatestResult();

            if(gamepad1.a) {
                pipeline=1;
                limelight3A.pipelineSwitch(pipeline);
                limelight3A.updatePythonInputs(min_x,min_y,max_x,max_y,class_id,10,10,10);
            }

            if(gamepad1.b){
                pipeline=0;
                limelight3A.pipelineSwitch(pipeline);
            }

            if(gamepad1.x){
                pipeline=2;
                limelight3A.pipelineSwitch(pipeline);
            }

            if(gamepad1.dpad_left && pipeline == 0){
                LLResultTypes.DetectorResult detectorResults = result.getDetectorResults().get(0);

                List<Double> pt1 = detectorResults.getTargetCorners().get(0);
                List<Double> pt2 = detectorResults.getTargetCorners().get(1);
                List<Double> pt3 = detectorResults.getTargetCorners().get(2);
                List<Double> pt4 = detectorResults.getTargetCorners().get(3);

                max_y = Math.max(pt1.get(1), Math.max(pt2.get(1), Math.max(pt3.get(1), pt4.get(1))));
                min_y = Math.min(pt1.get(1), Math.min(pt2.get(1), Math.min(pt3.get(1), pt4.get(1))));
                max_x = Math.max(pt1.get(0), Math.max(pt2.get(0), Math.max(pt3.get(0), pt4.get(0))));
                min_x = Math.min(pt1.get(0), Math.min(pt2.get(0), Math.min(pt3.get(0), pt4.get(0))));
                class_id = detectorResults.getClassId();
            }

            if(gamepad1.y){
                limelight3A.reloadPipeline();
            }

            telemetry.addData("Pipeline",pipeline);
            if(pipeline==0) Display_detector(result);
            if(pipeline==1) Display_python(result);
            if(pipeline==2) Display_color(result);
//            telemetry.addData("Valid",result.isValid());
            telemetry.addLine("TL:["+min_x+","+min_y+"]  BR:["+max_x+","+max_y+"]  Class:"+class_id);
            telemetry.update();

        }
    }

    public void Display_detector(LLResult result){
        try{
            for (LLResultTypes.DetectorResult detector : result.getDetectorResults()) {
                telemetry.addData("Class",detector.getClassId());
                telemetry.addData("Point 1",detector.getTargetCorners().get(0));
                telemetry.addData("Point 2",detector.getTargetCorners().get(1));
                telemetry.addData("Point 3",detector.getTargetCorners().get(2));
                telemetry.addData("Point 4",detector.getTargetCorners().get(3));
                telemetry.addLine("##########################");
            }
        }
        catch (Exception e){
            telemetry.addLine("##########################");
        }
    }

    public void Display_python(LLResult result){
        try{
            String out = "{"+result.getPythonOutput()[1]+","+result.getPythonOutput()[2]+"}";
            telemetry.addLine(out);
            telemetry.addData("Angle",result.getPythonOutput()[0]);
        }
        catch (Exception e){
            telemetry.addLine("##########################");
        }
    }

    public void Display_color(LLResult result){
        try{
            for (LLResultTypes.ColorResult colorResult : result.getColorResults()) {
                telemetry.addData("Point 1",colorResult.getTargetCorners().get(0));
                telemetry.addData("Point 2",colorResult.getTargetCorners().get(1));
                telemetry.addData("Point 3",colorResult.getTargetCorners().get(2));
                telemetry.addData("Point 4",colorResult.getTargetCorners().get(3));
                telemetry.addLine("##########################");
            }
        }
        catch (Exception e){
            telemetry.addLine("##########################");
        }
    }
}
