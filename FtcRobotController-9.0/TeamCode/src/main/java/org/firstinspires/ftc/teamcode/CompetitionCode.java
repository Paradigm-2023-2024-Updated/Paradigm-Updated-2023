//motor.setzeropowerbehavior(dcmotor.brake) use after hanging

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;


import com.qualcomm.hardware.bosch.BNO055IMU;                               //Importing of the files
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "CompetitionCode")
public class CompetitionCode extends OpMode {

    //GamePad 1
    DcMotor LFMotor, RBMotor, RFMotor, LBMotor;
    Servo claw;
    BNO055IMU imu;
    //testingbecauseofacommitingerror
    double angle;
    double joystickX;//x-value of the joystick
    double joystickY;//y-value of the joystick
    double LFPower, LBPower, RFPower, RBPower;
    double magnitude;
    double pivot;
    double max;//maximum value of one of the motor powers so we do not exceed 1 or -1


    //GamePad 2
    DcMotor linearSlide;
    DcMotor turnTable;



//_________________________________________________________________________________________

    @Override
    public void init() {

        //GamePad 1 Motor Setup
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        claw = hardwareMap.servo.get("claw");

        //GamePad 2 Motor and Servo Setup
        linearSlide = hardwareMap.dcMotor.get("linearSlide");
        turnTable = hardwareMap.dcMotor.get("turnTable");

        //Reverse Necessary Motors
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Imu Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);


        //Encoder Setup
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //___________________________________________________________________________________________

    @Override
    public void loop() {

        //GamePad 1

        //----------------

        //Mecanum Drive
        joystickX = gamepad1.left_stick_x; //joystick x-value
        joystickY = gamepad1.left_stick_y; //joystick y-value, inverted because controllers are weird
        pivot = (gamepad1.right_stick_x / 2.5); //pivot value, to be added or subtracted near the end
        magnitude = Range.clip(Math.hypot(joystickX, joystickY), 0, 1); //actual "length" of joystick, to be multiplied by sin value

        //based on the x and y input measurements from the joystick, an angle is radians is calculated
        angle = (getAngle() - Math.atan2(joystickY, joystickX));

        //unscaled power of each wheel; may exceed 1 or -1
        LFPower = 1.28*(Math.sin(angle + (Math.PI / 4)) * magnitude) + pivot;
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

        //Wheel Power Display
        telemetry.addData("Left Front", LFPower);//adds data to telemitry
        telemetry.addData("Right Front", RFPower);
        telemetry.addData("Left Back", LBPower);
        telemetry.addData("Right Back", RBPower);
        telemetry.addData("Robot Orientation",getAngle());
        telemetry.update();

        //----------------

        //Claw
        if (gamepad1.a) {
            claw.setPosition(0.1);
        } else {
            claw.setPosition(0.4);
        }

        //Turn Table Centralization
        if (gamepad1.x) {

            turnTable.setTargetPosition(0);
            turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turnTable.setPower(0.25);

            while (turnTable.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            turnTable.setPower(0);
            turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }


//__________________________________________________________________________________________________
        //GamePad 2

        //----------------

        //Linear Slide
        linearSlide.setPower(-(gamepad2.left_stick_y));

        //----------------

        //Turn Table
        turnTable.setPower((gamepad2.right_stick_x) / 3);

        //----------------
        //AndyMark Motor for linear slide (1120 revolutions/min)


        //Linear Slide Height 0
        if (gamepad2.x) {

            linearSlide.setTargetPosition(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.5);

            while (linearSlide.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Linear Slide Height 1
        if (gamepad2.a) {

            linearSlide.setTargetPosition(30);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.5);

            while (linearSlide.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        //Linear Slide Height 2
        if (gamepad2.b) {

            linearSlide.setTargetPosition(50);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.5);

            while (linearSlide.isBusy()) {

                telemetry.addData("Status:","Encoders in Progress...");

            }

            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        //Linear Slide Height 3
        if (gamepad2.y) {

            linearSlide.setTargetPosition(70);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.5);

            while (linearSlide.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public double getAngle() {
        return -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle);
    }

    public int tableValue() {

        return turnTable.getCurrentPosition();
    }

    public int slideValue() {
        return linearSlide.getCurrentPosition();
    }


    //mecanum works fine, adjustments needed
    //encoders
    //slides and turntable dont work
    //claw starts closed
}






///Odometry stuff (to be tested)

/*public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition  - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        // the robot has moved and turned a tiny bit between two measurements:
        double dtheta = cm_per_tick * ((dn2-dn1) / (LENGTH));
        double dx = cm_per_tick * ((dn1+dn2) / 2.0);
        double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));

        telemetrydx = dx;
        telemetrydy = dy;
        telemetrydh = dtheta;

        // small movement of the robot gets added to the field coordinate system:
        pos.h += dtheta / 2;
        pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
        pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
        pos.h += dtheta / 2;
        pos.h = normDiff(pos.h);
        }


 */