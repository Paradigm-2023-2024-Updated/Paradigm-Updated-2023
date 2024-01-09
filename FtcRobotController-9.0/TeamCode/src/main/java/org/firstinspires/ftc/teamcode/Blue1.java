package org.firstinspires.ftc.teamcode;
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




@Autonomous(name = "Blue 1")
public class Blue1 extends LinearOpMode {

    public final int WIDTH = 640;
    public final int HEIGHT = 360;
    public final int LEFT = (int) (WIDTH * 0.4);
    public final int RIGHT = (int) (WIDTH * 0.6);

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    DcMotor arm_m, linearActuator;
    Servo wrist,elbow;
    int tl_t, tr_t, bl_t, br_t; // _t for total
    int arm_t, la_t;

    private OpenCvCamera webcam;

    public void moveArm(int arm, int la) {
        arm_t += arm;
        la_t += la;

        arm_m.setTargetPosition(arm_t);
        arm_m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_m.setPower(0.5);

        linearActuator.setTargetPosition(la_t);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(0.5);

        telemetry.addData("arm",arm_t);
        telemetry.addData("LA",la_t);

        while (arm_m.isBusy() || linearActuator.isBusy()) {
            // Do nothing
            if (!opModeIsActive()) {
                break;
            }
        }

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
        LFMotor.setPower(0.5);

        RFMotor.setTargetPosition(tr_t);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setPower(0.5);

        LBMotor.setTargetPosition(-bl_t);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setPower(0.5);

        RBMotor.setTargetPosition(br_t);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setPower(0.5);

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
        arm_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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



        waitForStart();

        double position = ColorDetectionPipelineBlue1.centerX;

        telemetry.addData("Center X", position);
        telemetry.update();

        //0 is open, 0.7 is closed
        sleep(1000);
        elbow.setPosition(0.0);
        sleep(2000);
        elbow.setPosition(0.7);
        sleep(2000);

        //0.2 to 0.8 full range
        wrist.setPosition(0.2);
        sleep(2000);
        wrist.setPosition(1.1);
        sleep(2000);


        /*sleep(2000);
        wrist.setPosition(1.1);
        sleep(2000);
        moveArm(-10,0);*/


        if (position >= LEFT && position <= RIGHT) {
            telemetry.addData("Status", "Centered!");
            telemetry.update();
//            move(1000, 0, 0);
        } else if (position < LEFT) {
            // Turn Left
            telemetry.addData("Status", "Turn Left");
            telemetry.update();

//            move(1000, 0, -300);


        } else if (position > RIGHT) {
            // Turn right
            telemetry.addData("Status", "Turn Right");
            telemetry.update();

//            move(1000, 0, 300);
        }

    }

}

