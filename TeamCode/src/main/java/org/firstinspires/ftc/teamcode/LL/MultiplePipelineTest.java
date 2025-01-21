package org.firstinspires.ftc.teamcode.LL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MultiplePipelineTest extends LinearOpMode {
    Limelight3A limelight3A;
    public static final FtcDashboard dash = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();



        while (opModeIsActive()){

        }
    }
}
