package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="FieldCentric")
public class FieldCentricCode extends OpMode {

    //GamePad 1
    DcMotor LFMotor, RBMotor, RFMotor, LBMotor;
    BNO055IMU imu;
    double angle;
    double joystickX;//x-value of the joystick
    double joystickY;//y-value of the joystick
    double LFPower, LBPower, RFPower, RBPower;
    double magnitude;
    double pivot;
    double max;//maximum value of one of the motor powers so we do not exceed 1 or -1

    //________________________________________________________________________________


    @Override
    public void init() {

        //GamePad 1 Motor Setup
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        //Reverse Necessary Motors
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*//Imu Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


         */
    }

    //___________________________________________________________________________________________

    @Override
    public void loop() {

        //Mecanum Drive
        joystickX = gamepad1.left_stick_x; //joystick x-value
        joystickY = gamepad1.left_stick_y; //joystick y-value, inverted because controllers are weird
        pivot = (gamepad1.right_stick_x / 2.5); //pivot value, to be added or subtracted near the end
        magnitude = Range.clip(Math.hypot(joystickX, joystickY), 0, 1); //actual "length" of joystick, to be multiplied by sin value

        //based on the x and y input measurements from the joystick, an angle is radians is calculated
        angle = (getAngle() - Math.atan2(joystickY, joystickX));

        //unscaled power of each wheel; may exceed 1 or -1
        LFPower = (Math.sin(angle + (Math.PI / 4)) * magnitude) + pivot;
        RBPower = (Math.sin(angle + (Math.PI / 4)) * magnitude) - pivot;
        RFPower = (Math.sin(angle - (Math.PI / 4)) * magnitude) - pivot;
        LBPower = (Math.sin(angle - (Math.PI / 4)) * magnitude) + pivot;

        //maximum of abs value of all the motor values; divide each value by max so nothing exceeds 1 or -1
        if ((Math.abs(LFPower) > 1) || (Math.abs(RFPower) > 1) || (Math.abs(LBPower) > 1) || (Math.abs(RBPower) > 1)) {
            max = Math.max(Math.max(Math.abs(LFPower), Math.abs(RBPower)), Math.max(Math.abs(RFPower), Math.abs(LBPower)));

            LFPower = LFPower / max;
            RBPower = RBPower / max;
            RFPower = RFPower / max;
            LBPower = LBPower / max;
        }

        //Sets Motor Power
        LFMotor.setPower(-LFPower);
        LBMotor.setPower(-LBPower);
        RFMotor.setPower(-RFPower);
        RBMotor.setPower(-RBPower);

        //Robot Orientation Display
        telemetry.addData("Robot Orientation", getAngle());

        telemetry.update();


    }

    public double getAngle() {
        return -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle); //zxy
    }

}