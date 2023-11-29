//motor.setzeropowerbehavior(dcmotor.brake) use after hanging

package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
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
import org.firstinspires.ftc.teamcode.hardwareClasses.Arm;
import org.firstinspires.ftc.teamcode.hardwareClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Launcher;

@TeleOp(name = "CompetitionCode")
public class CompetitionCode extends OpMode {

    //define classes
    DriveTrain robot = new DriveTrain();
    Arm arm = new Arm();
    Launcher launcher = new Launcher();
    Intake intake = new Intake();

    //GamePad 1
    Servo claw;
    double angle;
    double joystickX;//x-value of the joystick
    double joystickY;//y-value of the joystick
    double pivot;


    //GamePad 2
    DcMotor linearSlide;
    DcMotor turnTable;



//_________________________________________________________________________________________

    @Override
    public void init() {

        robot.init(hardwareMap);

        //GamePad 1 Motor Setup
        claw = hardwareMap.servo.get("claw");

        launcher.init(hardwareMap);

        intake.init(hardwareMap);

        arm.init(hardwareMap);

        //GamePad 2 Motor and Servo Setup
        linearSlide = hardwareMap.dcMotor.get("linearSlide");
        turnTable = hardwareMap.dcMotor.get("turnTable");

        //Encoder Setup
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //___________________________________________________________________________________________

    @Override
    public void loop() {

        //Mecanum Drive
        joystickX = gamepad1.left_stick_x; //joystick x-value
        joystickY = -gamepad1.left_stick_y; //joystick y-value, inverted because controllers are weird
        pivot = (gamepad1.right_stick_x / 2.5); //pivot value, to be added or subtracted near the end
        angle = (robot.getAngle());

        robot.MecanumDrive(joystickX, joystickY, pivot,angle);

        intake.spinRoller();

        arm.rotateArm();

        arm.extendActuator();
        //----------------

        //Turn Table Centralization
        /*
        if (gamepad1.x) {

            turnTable.setTargetPosition(0);
            turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turnTable.setPower(0.25);

            while (turnTable.isBusy()) {
                telemetry.addData("Status:","Encoders in Progress...");

            }

            turnTable.setPower(0);
            turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }*/
        //----------------
        //AndyMark Motor for linear slide (1120 revolutions/min)








}