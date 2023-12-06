package org.firstinspires.ftc.teamcode.hardwareClasses;
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

   // BNO055IMU imu;

    public DriveTrain() {
    }

    public void init(HardwareMap hardwareMap) {
        //GamePad 1 Motor Setup
        this.LFMotor = hardwareMap.dcMotor.get("LFMotor");
        this.RBMotor = hardwareMap.dcMotor.get("RBMotor");
        this.RFMotor = hardwareMap.dcMotor.get("RFMotor");
        this.LBMotor = hardwareMap.dcMotor.get("LBMotor");

       /* imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters for the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // You may need to calibrate and save calibration data
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the IMU
        imu.initialize(parameters); */

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
        double RFPower = Range.clip(LRPower - FBPower - rotation, -1.0, 1.0);
        double LBPower = Range.clip(LRPower - FBPower + rotation, -1.0, 1.0);
        double RBPower = Range.clip(LRPower + FBPower - rotation, -1.0, 1.0);

        // Set motor powers
        this.LFMotor.setPower(LFPower);
        this.RFMotor.setPower(RFPower);
        this.LBMotor.setPower(LBPower);
        this.RBMotor.setPower(RBPower);
    }


    /*public double getAngle() {
        return -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle);
    }
    /*
     */


}
