package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    DcMotor arm, linearActuator;

    public Arm(){

    }

    public void init(HardwareMap hardwareMap) {

        this.arm = hardwareMap.dcMotor.get("arm");
        this.linearActuator = hardwareMap.dcMotor.get("linearActuator");

    }

    public void rotateArm(){
        this.arm.setPower(gamepad1.right_stick_y);
    }

    public void extendActuator(double power){
        this.linearActuator.setPower(power);
    }

    public void retractActuator(double power){
        this.linearActuator.setPower(-power);
    }
}
