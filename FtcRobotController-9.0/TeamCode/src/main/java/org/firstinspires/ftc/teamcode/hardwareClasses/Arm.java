package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    DcMotor arm, linearActuator;

    Servo wrist,elbow;
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

    public void moveArm(int arm, int la) {
        arm_t += arm;
        la_t += la;

        this.arm.setTargetPosition(arm_t);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(0.5);

        linearActuator.setTargetPosition(la_t);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setPower(0.5);

        telemetry.addData("arm",arm_t);
        telemetry.addData("LA",la_t);

        while (this.arm.isBusy() || linearActuator.isBusy()) {
            // Do nothing
            if (!linearOpMode.opModeIsActive()) {
                break;
            }
        }

        this.arm.setPower(0);
        linearActuator.setPower(0);
    }

    public void rotateArm(double power){
        this.arm.setPower(.4*power);
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
