package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RBMotor, RFMotor;

    double radius = 5;
    double circumference = 2 * 3.1415926535 * radius;

    double ticksPerRev = 540;


    @Override
    public void runOpMode() throws InterruptedException {
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LFMotor.setTargetPosition((int) ticksPerRev + LFMotor.getCurrentPosition());
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(0.7);

        RFMotor.setTargetPosition((int) ticksPerRev + LFMotor.getCurrentPosition());
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setPower(0.7);

        LBMotor.setTargetPosition((int) ticksPerRev + LFMotor.getCurrentPosition());
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setPower(0.7);

        RBMotor.setTargetPosition((int) ticksPerRev + LFMotor.getCurrentPosition());
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setPower(0.7);

        telemetry.addData("LFMotor", LFMotor.getCurrentPosition());
        telemetry.addData("RFMotor", RFMotor.getCurrentPosition());
        telemetry.addData("LBMotor", LBMotor.getCurrentPosition());
        telemetry.addData("RBMotor", RBMotor.getCurrentPosition());

        telemetry.update();

    }
}