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



        arm.setTargetPosition(a);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);


        while (arm.isBusy()) {
            // Do nothing

        }


        arm.setPower(0);

    }
    public void moveActuator(int la) {



        linearActuator.setTargetPosition(la);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(0.5);


        while (linearActuator.isBusy()) {
            // Do nothing

        }


        linearActuator.setPower(0);

    }
    public void rotateArm(double power){
        this.arm.setPower(-.4*power);
    }

    public void rotateActuator(double power){
        this.linearActuator.setPower(0.9*power);
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
