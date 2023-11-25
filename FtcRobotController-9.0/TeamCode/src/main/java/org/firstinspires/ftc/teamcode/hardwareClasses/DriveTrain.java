package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
public class DriveTrain {

    DcMotor LFMotor, RBMotor, RFMotor, LBMotor;

    public DriveTrain() {
    }

    public void init(HardwareMap hardwareMap) {
        //GamePad 1 Motor Setup
        this.LFMotor = hardwareMap.dcMotor.get("LFMotor");
        this.RBMotor = hardwareMap.dcMotor.get("RBMotor");
        this.RFMotor = hardwareMap.dcMotor.get("RFMotor");
        this.LBMotor = hardwareMap.dcMotor.get("LBMotor");

        //Reverse Necessary Motors (flip this)
        this.RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void MecanumDrive(double LRPower, double FBPower, double PHIPower) {
        // convert to motor powers
        double LFPower = FBPower + LRPower + PHIPower;
        double LBPower = FBPower - LRPower + PHIPower;
        double RFPower = FBPower - LRPower - PHIPower;
        double RBPower = FBPower + LRPower - PHIPower;

        // clip the motor powers/scale/wahterver
        //maximum of abs value of all the motor values; divide each value by max so nothing exceeds 1 or -1
        if ((Math.abs(LFPower) > 1) || (Math.abs(RFPower) > 1) || (Math.abs(LBPower) > 1) || (Math.abs(RBPower) > 1)) {
            double max = Math.max(Math.max(Math.abs(LFPower), Math.abs(RBPower)), Math.max(Math.abs(RFPower), Math.abs(LBPower)));

            LFPower = LFPower / max;
            RBPower = RBPower / max;
            RFPower = RFPower / max;
            LBPower = LBPower / max;
        }

        // set powers ( configure motor directions in init)
        this.LFMotor.setPower(-LFPower);//sets the motors power
        this.LBMotor.setPower(-LBPower);
        this.RFMotor.setPower(-RFPower);
        this.RBMotor.setPower(-RBPower);
    }


}
