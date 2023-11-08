package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;                               //Importing of the files
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

@TeleOp(name = "TeleopCode24")
public class TeleopCode24 extends OpMode {

    DcMotor LFM, RBM, RFM, LBM, roller, extend, elbow;
    Servo paper, claw, wrist, push, lift;
    BNO055IMU imu;
    double angle;
    double joystickX;
    double joystickY;
    double LFP, LBP, RFP, RBP;
    double magnitude;
    double pivot;
    double max;
    double a;
    // indivitual motor movement or set hights. flip button? idk  reset button    grab    

//_________________________________________________________________________________________

    @Override
    public void init() {

        LFM = hardwareMap.dcMotor.get("LFM");
        RBM = hardwareMap.dcMotor.get("RBM");
        RFM = hardwareMap.dcMotor.get("RFM");
        LBM = hardwareMap.dcMotor.get("LBM");

        roller = hardwareMap.dcMotor.get("Roller");
        extend = hardwareMap.dcMotor.get("Extend");
        elbow = hardwareMap.dcMotor.get("Elbow");

        paper = hardwareMap.servo.get("Paper");
        claw = hardwareMap.servo.get("Claw");
        wrist = hardwareMap.servo.get("Wrist");
        push = hardwareMap.servo.get("Push");
        lift = hardwareMap.servo.get("Lift");

        paper.setPosition(1);

    }

//___________________________________________________________________________________________

    @Override
    public void loop() {

        //Mecanum Drive
        joystickX = gamepad1.left_stick_x; //joystick x-value
        joystickY = -gamepad1.left_stick_y; //joystick y-value, inverted because controllers are weird
        pivot = (gamepad1.right_stick_x / 2.5); //pivot value, to be added or subtracted near the end
        magnitude = Range.clip(Math.hypot(joystickX, joystickY), 0, 1); //actual "length" of joystick, to be multiplied by sin value

        //based on the x and y input measurements from the joystick, an angle is radians is calculated
        angle = (getAngle() - Math.atan2(joystickY, joystickX));

        //unscaled power of each wheel; may exceed 1 or -1
        LFP = (Math.sin(angle + (Math.PI / 4)) * magnitude) + pivot;
        RBP = (Math.sin(angle + (Math.PI / 4)) * magnitude) - pivot;
        RFP = (Math.sin(angle - (Math.PI / 4)) * magnitude) - pivot;
        LBP = (Math.sin(angle - (Math.PI / 4)) * magnitude) + pivot;

        //maximum of abs value of all the motor values; divide each value by max so nothing exceeds 1 or -1
        if ((Math.abs(LFP) > 1) || (Math.abs(RFP) > 1) || (Math.abs(LBP) > 1) || (Math.abs(RBP) > 1)) {
            max = Math.max(Math.max(Math.abs(LFP), Math.abs(RBP)), Math.max(Math.abs(RFP), Math.abs(LBP)));

            LFP = LFP / max;
            RBP = RBP / max;
            RFP = RFP / max;
            LBP = LBP / max;
        }

        LFM.setPower(LFP);//sets the motors power
        LBM.setPower(LBP);
        RFM.setPower(RFP);
        RBM.setPower(RBP);

        telemetry.addData("Left Front", LFP);
        telemetry.addData("Right Front", RFP);
        telemetry.addData("Left Back", LBP);
        telemetry.addData("Right Back", RBP);
        telemetry.update();

        if (gamepad1.y) {
            paper.setPosition(1); // need to set servo pos ***************************************
        }
        if (gamepad2.left_stick_button) {
            claw.setPosition(1);
        }
        if (gamepad2.right_stick_button) {
            claw.setPosition(0);
        }
        if (gamepad2.left_bumper) {
            a +=0.1;
        }
        if (gamepad2.right_bumper) {
            a -=0.1;
        }
        if (a > 1) {
            a = 1;
        }
        if (a < 0) {
            a = 0;
        }
        wrist.setPosition(a);
        elbow.setPower(gamepad2.left_stick_y);
        extend.setPower(gamepad2.right_stick_y);
        if (gamepad2.a) {
            extend.setTargetPosition(0);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setPower(0.5);
            elbow.setTargetPosition(0);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(0.5);

            while (extend.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }
            while (elbow.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            extend.setPower(0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setPower(0);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wrist.setPosition(0);
            claw.setPosition(0);
        }
        if (gamepad1.b) {
            lift.setPosition(1);
        }else {
            lift.setPosition(0);
        }

    }

    public double getAngle() {
        return -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle);
    }


}
