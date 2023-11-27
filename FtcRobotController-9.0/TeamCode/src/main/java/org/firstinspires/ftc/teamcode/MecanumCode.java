package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.hardwareClasses.Arm;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Launcher;

//hardware classes (added by max, if they're bad blame him)


@TeleOp(name = "MecanumDrive")
public class MecanumCode extends OpMode {

    //define classes
    DriveTrain robot = new DriveTrain();
    Arm arm = new Arm();
    Launcher launcher = new Launcher();
    Intake intake = new Intake();


    //GamePad 1
    DcMotor LFMotor, RBMotor, RFMotor, LBMotor;
    double angle;
    double joystickX;//x-value of the joystick
    double joystickY;//y-value of the joystick
    double LFPower, LBPower, RFPower, RBPower;
    double magnitude;
    double pivot;
    double max;//maximum value of one of the motor powers so we do not exceed 1 or -1

//_________________________________________________________________________________________

    @Override
    public void init() {

        robot.init(hardwareMap);

        /*
        //GamePad 1 Motor Setup
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        //Reverse Necessary Motors
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         */
    }

//___________________________________________________________________________________________

    @Override
    public void loop() {


        //Mecanum Drive
        joystickX = gamepad1.left_stick_x; //joystick x-value
        joystickY = -gamepad1.left_stick_y; //joystick y-value, inverted because controllers are weird
        pivot = (gamepad1.right_stick_x / 2.5); //pivot value, to be added or subtracted near the end


        // same this as all of the code above
        robot.MecanumDrive(joystickX, joystickY, pivot,0);


    }


}