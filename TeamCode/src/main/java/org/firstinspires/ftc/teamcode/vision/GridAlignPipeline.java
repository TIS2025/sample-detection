package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

public class GridAlignPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    public Point calcProjection(Point x,Mat rot, Mat trans){
        return new Point(0,0);
    }
}
