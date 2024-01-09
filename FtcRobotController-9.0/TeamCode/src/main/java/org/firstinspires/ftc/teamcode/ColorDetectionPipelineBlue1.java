package org.firstinspires.ftc.teamcode;

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
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ColorDetectionPipelineBlue1 extends OpenCvPipeline {

    public static double centerX;
    public static double centerY;

    @Override
    public Mat processFrame(Mat input) {
        // Process the frame using OpenCV
        Imgproc.GaussianBlur(input, input, new Size(9, 9), 0);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        // Define the HSV range for blue color
        Scalar lowerBlue = new Scalar(90, 100, 100);
        Scalar upperBlue = new Scalar(130, 255, 255);

        Mat mask = new Mat();
        Core.inRange(input, lowerBlue, upperBlue, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 2);  // Use Scalar(0, 0, 255) for blue color

        if (!contours.isEmpty()) {
            for (MatOfPoint contour : contours) {
                double contourArea = Imgproc.contourArea(contour);
                if (contourArea >= 200) {
                    MatOfPoint largestContour = Collections.max(contours, Comparator.comparing(Imgproc::contourArea));
                    Moments moments = Imgproc.moments(largestContour);
                    centerX = moments.m10 / moments.m00;
                    centerY = moments.m01 / moments.m00;
                    // Draw a circle at the center point
                    Imgproc.circle(input, new Point(centerX, centerY), 10, new Scalar(0, 0, 255), -1);  // Use Scalar(0, 0, 255) for blue color
                } else {
                    centerX = -1;
                    centerY = -1;
                }
            }
        }

        mask.release();
        hierarchy.release();

        // Convert back to RGB color space for display
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

        return input;
    }
}
