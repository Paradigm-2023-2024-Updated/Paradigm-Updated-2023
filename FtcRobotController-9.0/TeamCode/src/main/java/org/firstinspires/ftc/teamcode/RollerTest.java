package org.firstinspires.ftc.teamcode; // Replace with your package name

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Roller")
public class RollerTest extends OpMode {

    private DcMotor motor; // Define your motor here

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor"); // Replace "motorName" with the name configured on your robot controller
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Check if button 'A' is pressed
        motor.setPower(-(gamepad1.left_stick_y));
        //motor.setPower(-1);


        telemetry.addData("Button A Pressed", gamepad1.a);
        telemetry.update();
    }
}
