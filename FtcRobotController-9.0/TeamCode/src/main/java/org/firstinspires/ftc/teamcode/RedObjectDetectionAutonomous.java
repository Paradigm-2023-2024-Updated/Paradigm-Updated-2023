package org.firstinspires.ftc.teamcode;
//REPLACE XCOORD WITH  METHOD CALL
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;



// Comments about direction (Alden)
// LA - pos is in/retract
// ARM - pos is grabber down
// Elbow:
// 0.5 - Parallel to screws
// 0.75 - Parallel to arm
// 1 - Opposite from screws




@Autonomous(name = "Contour")
public class RedObjectDetectionAutonomous extends LinearOpMode {

    public final int WIDTH = 640;
    public final int HEIGHT = 360;
    public final int LEFT = (int) (WIDTH * 0.4);
    public final int RIGHT = (int) (WIDTH * 0.6);

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    DcMotor arm_m, linearActuator;
    Servo wrist,elbow;
    int tl_t, tr_t, bl_t, br_t; // _t for total
    int arm_t = 0, la_t = 0;

    private OpenCvCamera webcam;

    public void moveArm(int arm, int la, boolean ignore) {
        arm_t += arm;
        la_t += la;

        arm_m.setTargetPosition(arm_t);
        arm_m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_m.setPower(0.85);

        linearActuator.setTargetPosition(la_t);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(0.8);

        telemetry.addData("MA arm",arm_t);
        telemetry.addData("MA LA",la_t);
        telemetry.update();
        sleep(1000);

        while (arm_m.isBusy() || linearActuator.isBusy()) {
            // Do nothing
            if (!opModeIsActive() && !ignore) {
                break;
            }
            telemetry.addData("MA arm update", arm_m.getCurrentPosition());
            telemetry.addData("MA la update", linearActuator.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("MA done",la_t);
        telemetry.update();

        arm_m.setPower(0);
        linearActuator.setPower(0);
    }

    public void move(int forward, int strafe, int turn) {
        tl_t += forward + strafe + turn;
        tr_t += forward - strafe - turn;
        bl_t += forward - strafe + turn;
        br_t += forward + strafe - turn;

        LFMotor.setTargetPosition(-tl_t);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(0.7);

        RFMotor.setTargetPosition(tr_t);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setPower(0.7);

        LBMotor.setTargetPosition(-bl_t);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setPower(0.7);

        RBMotor.setTargetPosition(br_t);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setPower(0.7);

        telemetry.addData("tl",tl_t);
        telemetry.addData("tr",tr_t);
        telemetry.addData("bl",bl_t);
        telemetry.addData("br",br_t);
        telemetry.update();
        while (LFMotor.isBusy() || RFMotor.isBusy() || RBMotor.isBusy() || LBMotor.isBusy()) {
            // Do nothing
            if (!opModeIsActive()) {
                break;
            }
            telemetry.addData("tl cur",LFMotor.getCurrentPosition());
            telemetry.addData("tr cur",RFMotor.getCurrentPosition());
            telemetry.addData("bl cur",LBMotor.getCurrentPosition());
            telemetry.addData("br cur",RBMotor.getCurrentPosition());
            telemetry.addData("tl on",LFMotor.isBusy());
            telemetry.addData("tr on",RFMotor.isBusy());
            telemetry.addData("bl on",LBMotor.isBusy());
            telemetry.addData("br on",RBMotor.isBusy());
            telemetry.update();
        }

        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_m = hardwareMap.dcMotor.get("arm");
        arm_m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        arm_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wrist = hardwareMap.servo.get("wrist");
        elbow = hardwareMap.servo.get("elbow");




        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(new ColorDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {

                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
                sleep(2000);
            }

            @Override
            public void onError(int errorCode) {

            }
        });




        sleep(500);
        telemetry.addData("Wrist", 0.4);
        telemetry.update();
        wrist.setPosition(0.4); // open
        sleep(500);

        telemetry.addData("Arm", -1200);
        telemetry.update();
        moveArm(-1200,0, true);
        sleep(500);

        telemetry.addData("Elbow", 0.5);
        telemetry.update();
        sleep(10);
        elbow.setPosition(0.5); // facing down

        telemetry.addData("READY TO GO", 0);
        telemetry.update();

        waitForStart();


        sleep(10);
        telemetry.addLine("STRAFE");
        telemetry.update();
        move(0, 1000, 0);
        telemetry.addLine("SPIN");
        telemetry.update();
        sleep(10);
        move(0, 0, 950);

        sleep(10);
        telemetry.addLine("BACK");
        telemetry.update();
        move(-690, 0, 0);

        telemetry.addLine("ALIGN");
        telemetry.update();
        move(0, 80, 0);


//        double position = ColorDetectionPipeline.centerX;
        double position = -1.0;
        while (position < 0.0) {
            telemetry.addData("TRY POS", position);
            telemetry.update();
            position = ColorDetectionPipeline.centerX;
            sleep(10);
        }

        sleep(200);
        telemetry.addData("Center X", position);
        telemetry.update();
        sleep(500);
        move(200, 0, 0);





        if (position >= LEFT && position <= RIGHT) {
            telemetry.addData("Status", "Centered!");
            telemetry.update();

            move(400, 0, 0);
            move(0, -220, 0);
            //-320 for red 2
            move(550, 0, 0);
//-----------------------------------------------------------


            telemetry.addData("LA", 1500);
            telemetry.update();
            moveArm(0,3100, false);



            telemetry.addData("Arm", 850);
            telemetry.update();
            moveArm(850,0, false);




            telemetry.addData("Wrist", 1.1);
            telemetry.update();
            wrist.setPosition(1.1); // open
            sleep(1000);

            moveArm(-850,0, false);
            moveArm(0,-3100, false);


            //move to backstage
            move(-1200,0,0);
            move(0,0,950);
            move(4000,0,0);
            //1400 for red 1


        } else if (position < LEFT) {
            // Turn Left
            telemetry.addData("Status","Turn Left");
            telemetry.update();



            move(600, 0, 0);
            move(0, 0, -950);
            move(0, 500, 0);

            //----------------------------------------


            telemetry.addData("LA", 1500);
            telemetry.update();
            moveArm(0,3300, false);



            telemetry.addData("Arm", 850);
            telemetry.update();
            moveArm(850,0, false);



            telemetry.addData("Wrist", 1.1);
            telemetry.update();
            wrist.setPosition(1.1); // open
            sleep(1000);

            moveArm(-850,0, false);
            moveArm(0,-3300, false);

            //move to backstage
            move(0, -1400, 0);
            move(-4000,0,0);
            //-1400 for red 2



        } else if (position > RIGHT) {
            // Turn right
            telemetry.addData("Status", "Turn Right");
            telemetry.update();


            move(600, 0, 0);
            move(0, 0, 950);
            move(0, -500, 0);

            //--------------------------------------------


            telemetry.addData("LA", 1500);
            telemetry.update();
            moveArm(0,3300, false);


            telemetry.addData("Arm", 850);
            telemetry.update();
            moveArm(850,0, false);


            telemetry.addData("Wrist", 0.4);
            telemetry.update();
            wrist.setPosition(1.1); // open
            sleep(1000);

            moveArm(-850,0, false);
            moveArm(0,-3300, false);

            //move to backstage
            move(0, 1400, 0);
            move(4000,0,0);
            //1400 for red 1

        }

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
                if (contourArea >= 300) {
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
