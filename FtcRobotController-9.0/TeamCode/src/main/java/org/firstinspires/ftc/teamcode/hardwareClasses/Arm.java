package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    DcMotor arm, linearActuator;
    Servo wrist;

    public Arm(){

    }

    public void init(HardwareMap hardwareMap) {

        this.arm = hardwareMap.dcMotor.get("arm");
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.linearActuator = hardwareMap.dcMotor.get("linearActuator");
        this.wrist = hardwareMap.servo.get("wrist");

    }

    public void rotateArm(double power){
        this.arm.setPower(power);
    }

    public void extendActuator(double power){
        this.linearActuator.setPower(power);
    }

    public void retractActuator(double power){
        this.linearActuator.setPower(-power);
    }

    public void stopActuator(){
        this.linearActuator.setPower(0);
    }

    public void grab(){
        this.wrist.setPosition(1);
    }

    public void release(){
        this.wrist.setPosition(0);
    }
}
