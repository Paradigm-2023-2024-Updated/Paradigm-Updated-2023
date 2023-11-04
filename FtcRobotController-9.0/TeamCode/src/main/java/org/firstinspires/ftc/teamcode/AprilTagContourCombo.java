package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "TagContour")
public class AprilTagContourCombo extends LinearOpMode {

    private double centerX = -1; // Initialize to -1 if no valid contour is found
    private double centerY = -1; // Initialize to -1 if no valid contour is found


    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {


        //Webcam Set Up
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipelineFinal());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {

                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //April Tag Set Up
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480)) //adjust width and height as needed
                .build();



        centerX = ColorDetectionPipeline.centerX;
        centerY = ColorDetectionPipeline.centerY;
        telemetry.addData("Center X", ColorDetectionPipeline.centerX);
        telemetry.addData("Center Y", ColorDetectionPipeline.centerY);

        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {



            if (centerX >= 305 && centerX <= 335) {
                telemetry.addData("Status", "Centered!");
                break;

            } else if (centerX < 305) {
                // Turn right
                telemetry.addData("Status", "Turn Left");
                // Add code to turn the robot right
            } else if (centerX > 335) {
                // Turn left
                telemetry.addData("Status", "Turn Right");
                // Add code to turn the robot left
            }

            telemetry.addData("Center X", ColorDetectionPipeline.centerX);
            telemetry.addData("Center Y", ColorDetectionPipeline.centerY);

            telemetry.update();

        }

        //webcam.stopStreaming();
        //webcam.closeCameraDevice();
    }

    class ColorDetectionPipelineFinal extends OpenCvPipeline {

        private boolean centerPointFound = false;
        public double centerX;
        public double centerY;

        @Override
        public Mat processFrame(Mat input) {
            if (!centerPointFound) {
                // Process the frame using OpenCV
                Imgproc.GaussianBlur(input, input, new Size(9, 9), 0);
                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

                Scalar lowerRed1 = new Scalar(0, 100, 100);
                Scalar upperRed1 = new Scalar(10, 255, 255);

                Scalar lowerRed2 = new Scalar(120, 100, 100);
                Scalar upperRed2 = new Scalar(180, 255, 255);

                Mat mask1 = new Mat();
                Core.inRange(input, lowerRed1, upperRed1, mask1);

                Mat mask2 = new Mat();
                Core.inRange(input, lowerRed2, upperRed2, mask2);

                Mat mask = new Mat();
                Core.bitwise_or(mask1, mask2, mask);

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);

                if (!contours.isEmpty()) {
                    for (MatOfPoint contour : contours) {
                        double contourArea = Imgproc.contourArea(contour);
                        if (contourArea >= 300) {
                            MatOfPoint largestContour = Collections.max(contours, Comparator.comparing(Imgproc::contourArea));
                            Moments moments = Imgproc.moments(largestContour);
                            centerX = moments.m10 / moments.m00;
                            centerY = moments.m01 / moments.m00;
                            centerPointFound = true; // Set the flag to indicate that the center point has been found
                            // Draw a circle at the center point
                            Imgproc.circle(input, new Point(centerX, centerY), 10, new Scalar(255, 0, 0), -1);
                        }
                    }
                }

                mask1.release();
                mask2.release();
                mask.release();
                hierarchy.release();

                // Convert back to RGB color space for display
                Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
            }

            return input;
        }
    }

}


