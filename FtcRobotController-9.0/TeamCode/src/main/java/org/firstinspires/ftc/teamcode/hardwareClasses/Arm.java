package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    DcMotor arm, linearActuator;

    public Servo wrist,elbow;
    int arm_t, la_t;




    public Arm(){

    }

    public void init(HardwareMap hardwareMap) {

        this.arm = hardwareMap.dcMotor.get("arm");
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.linearActuator = hardwareMap.dcMotor.get("linearActuator");

        this.wrist = hardwareMap.servo.get("wrist");
        this.elbow = hardwareMap.servo.get("elbow");




    }

    public void moveArm(int a) {

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setTargetPosition(a);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);


        while (arm.isBusy()) {
            // Do nothing

        }


        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void moveActuator(int la) {

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearActuator.setTargetPosition(la);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(0.9);


        while (linearActuator.isBusy()) {
            // Do nothing


        }


        linearActuator.setPower(0);
        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void setFinalPosition(){
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearActuator.setTargetPosition(0);
        arm.setTargetPosition(-800);

        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearActuator.setPower(0.9);
        arm.setPower(0.9);

        while (linearActuator.isBusy() || arm.isBusy()) {
            // Wait for motors to reach target positions
        }

        // Stop the motors when the target positions are reached
        linearActuator.setPower(0);
        arm.setPower(0);

        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateArm(double power){

        this.arm.setPower(.4*power);
    }

    public void rotateActuator(double power){
        this.linearActuator.setPower(-0.9*power);
    }


    public void grab(){
        this.wrist.setPosition(1);
    }

    public void release(){
        this.wrist.setPosition(0.5);
    }

    public void elbowUp(double up){
        this.elbow.setPosition(up);
    }

    public void elbowDown(double down){
        this.elbow.setPosition(down);
    }




}
