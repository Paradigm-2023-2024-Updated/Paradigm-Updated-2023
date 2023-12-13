package org.firstinspires.ftc.teamcode;
//REPLACE XCOORD WITH  METHOD CALL
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Autonomous(name = "Contour")
public class RedObjectDetectionAutonomous extends LinearOpMode {

    DcMotor LFMotor,LBMotor,RFMotor,RBMotor;

    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        WebcamName webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {

                webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
                sleep(2000);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        double position = ColorDetectionPipeline.centerX;
        double positionY = ColorDetectionPipeline.centerY;

        telemetry.addData("Center X", position);
        telemetry.addData("Center Y",positionY);

        if (position > 290 && position < 350) {
            telemetry.addData("Status", "Centered!");
        } else if (position < 290) {
            // Turn Left
            telemetry.addData("Status", "Turn Left");

            LFMotor.setTargetPosition(-200);
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LFMotor.setPower(1);

            RFMotor.setTargetPosition(-200);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setPower(1);


            LBMotor.setTargetPosition(-200);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setPower(1);


            RBMotor.setTargetPosition(-200);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setPower(1);

            while (LFMotor.isBusy()&& RFMotor.isBusy()&& RBMotor.isBusy()&& LBMotor.isBusy()) {
                // Do nothing
            }


            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Add code to turn the robot right
            } else if (position > 350) {
            // Turn right
            telemetry.addData("Status", "Turn Right");
            LFMotor.setTargetPosition(200);
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LFMotor.setPower(1);

            RFMotor.setTargetPosition(200);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setPower(1);


            LBMotor.setTargetPosition(200);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setPower(1);


            RBMotor.setTargetPosition(200);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setPower(1);

            while (LFMotor.isBusy()&& RFMotor.isBusy()&& RBMotor.isBusy()&& LBMotor.isBusy()) {
                // Do nothing
            }


            LFMotor.setPower(0);
            RFMotor.setPower(0);
            LBMotor.setPower(0);
            RBMotor.setPower(0);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Add code to turn the robot left
            }

            telemetry.update();
        }


    }

class ColorDetectionPipeline extends OpenCvPipeline {

    public static double centerX;
    public static double centerY;

    @Override
    public Mat processFrame(Mat input) {
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
                if (contourArea >= 200) {
                    MatOfPoint largestContour = Collections.max(contours, Comparator.comparing(Imgproc::contourArea));
                    Moments moments = Imgproc.moments(largestContour);
                    centerX = moments.m10 / moments.m00;
                    centerY = moments.m01 / moments.m00;
                    // Draw a circle at the center point
                    Imgproc.circle(input, new Point(centerX, centerY), 10, new Scalar(255, 0, 0), -1);
                } else {
                    centerX = -1;
                    centerY = -1;
                }
            }
        }

        mask1.release();
        mask2.release();
        mask.release();
        hierarchy.release();

        // Convert back to RGB color space for display
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

        return input;
    }
}

/*class ColorDetectionPipeline extends OpenCvPipeline {

    public static double centerX;
    public static double centerY;

    public static int position = 0;

    @Override
    public Mat processFrame(Mat input) {
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

                    // Calculate the position based on the center point with a smaller buffer
                    double imageCenterX = input.width() / 2.0;
                    double positionError = centerX - imageCenterX;
                    double buffer = 5.0; // Smaller buffer size

                    if (positionError < -buffer) {
                        position = -1; // Contour center is to the left
                    } else if (positionError > buffer) {
                        position = 1; // Contour center is to the right
                    } else {
                        position = 0; // Contour center is within the buffer, consider it centered
                    }

                    // Draw a circle at the center point
                    Imgproc.circle(input, new Point(centerX, centerY), 10, new Scalar(255, 0, 0), -1);
                } else {
                    position = 2; // No significant contour found
                }
            }
        }

        mask1.release();
        mask2.release();
        mask.release();
        hierarchy.release();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

        return input;
    }*/
