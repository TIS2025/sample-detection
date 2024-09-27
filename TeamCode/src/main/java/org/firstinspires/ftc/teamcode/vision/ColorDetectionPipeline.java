package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ColorDetectionPipeline extends OpenCvPipeline {
    // Preallocate matrices for reuse
    private final Mat rgbFrame = new Mat();
    private final Mat hsvFrame = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private final Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Preprocess frames and find contours for each color
        List<MatOfPoint> red_contours = findContours(preprocessFrame(input, ColorConst.lowRed, ColorConst.highRed));
        List<MatOfPoint> blue_contours = findContours(preprocessFrame(input, ColorConst.lowBlue, ColorConst.highBlue));
        List<MatOfPoint> yellow_contours = findContours(preprocessFrame(input, ColorConst.lowYellow, ColorConst.highYellow));

        // Draw contours on the input frame
        drawContours(input, red_contours, "RED");
        drawContours(input, yellow_contours, "YELLOW");
        drawContours(input, blue_contours, "BLUE");

        return input;
    }

    private Mat preprocessFrame(Mat input, Scalar low, Scalar high) {
        // Convert to RGB and then to HSV
        Imgproc.cvtColor(input, rgbFrame, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(rgbFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Apply the color mask
        Core.inRange(hsvFrame, low, high, mask);

        // Apply morphology operations to clean up the mask
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        return mask;
    }

    private List<MatOfPoint> findContours(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    private void drawContours(Mat input, List<MatOfPoint> contours, String color) {
        // Set the contour color based on the string
        Scalar obj_color = (Objects.equals(color, "RED")) ? new Scalar(255, 0, 0) :
                (Objects.equals(color, "BLUE")) ? new Scalar(0, 0, 255) :
                        (Objects.equals(color, "YELLOW")) ? new Scalar(255, 255, 0) : new Scalar(0, 0, 0);

        // Draw each contour and its centroid
        for (MatOfPoint contour : contours) {
            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            if (moments.get_m00() > 400) {
                // Draw the contour and centroid
                Imgproc.drawContours(input, contours, contours.indexOf(contour), obj_color, 2);
                Imgproc.circle(input, new Point(cX, cY), 2, new Scalar(0, 255, 0), -1);
            }
        }
    }
}
