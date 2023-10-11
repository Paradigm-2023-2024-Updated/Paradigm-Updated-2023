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

    DcMotor LFM, RBM, RFM, LBM;
    BNO055IMU imu;
    double angle;
    double joystickX;
    double joystickY;
    double LFP, LBP, RFP, RBP;
    double magnitude;
    double pivot;
    double max;
    // indivitual motor movement or set hights. flip button? idk  reset button    grab    

//_________________________________________________________________________________________

    @Override
    public void init() {

        LFM = hardwareMap.dcMotor.get("LFM");
        RBM = hardwareMap.dcMotor.get("RBM");
        RFM = hardwareMap.dcMotor.get("RFM");
        LBM = hardwareMap.dcMotor.get("LBM");

        //RFM.setDirection(DcMotorSimple.Direction.REVERSE);
        //RBM.setDirection(DcMotorSimple.Direction.REVERSE);
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

    }

    public double getAngle() {
        return -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle);
    }


}
