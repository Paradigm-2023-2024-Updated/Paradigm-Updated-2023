package org.firstinspires.ftc.teamcode.hardwareClasses;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain {

    DcMotor LFMotor, RBMotor, RFMotor, LBMotor;
    int tl_t, tr_t, bl_t, br_t; // _t for total


    public DriveTrain() {
    }

    public void init(HardwareMap hardwareMap) {
        //GamePad 1 Motor Setup
        this.LFMotor = hardwareMap.dcMotor.get("LFMotor");
        this.RBMotor = hardwareMap.dcMotor.get("RBMotor");
        this.RFMotor = hardwareMap.dcMotor.get("RFMotor");
        this.LBMotor = hardwareMap.dcMotor.get("LBMotor");


        //Reverse Necessary Motors (flip this)
        this.LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void driveFieldCentric(double LRPower, double FBPower, double rotation) {
        // Get robot heading in radians
        //double heading = Math.toRadians(imu.getAngularOrientation().firstAngle);

        // Calculate the new x and y components after applying rotation
      //  double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        //double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

        // Calculate motor powers
        double LFPower = Range.clip(LRPower + FBPower + rotation, -1.0, 1.0);
        double RFPower = Range.clip(-LRPower + FBPower - rotation, -1.0, 1.0);
        double LBPower = Range.clip(-LRPower + FBPower + rotation, -1.0, 1.0);
        double RBPower = Range.clip(LRPower + FBPower - rotation, -1.0, 1.0);

        // Set motor powers
        this.LFMotor.setPower(.8*LFPower);
        this.RFMotor.setPower(.8*RFPower);
        this.LBMotor.setPower(.8*LBPower);
        this.RBMotor.setPower(.8*RBPower);
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
            if (!linearOpMode.opModeIsActive()) {
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




    //Create method to run robot specific number of ticks


}
