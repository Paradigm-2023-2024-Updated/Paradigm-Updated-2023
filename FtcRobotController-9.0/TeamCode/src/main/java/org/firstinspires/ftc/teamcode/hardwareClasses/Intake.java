package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor roller;
    private boolean isMotorOn = false;

    public Intake(){

    }

    public void init(HardwareMap hardwareMap) {

        this.roller = hardwareMap.dcMotor.get("roller");

    }

    public void spinRoller() {
        // Check if the button is pressed
                // Button is pressed
        this.roller.setPower(-0.65);

        }
    public void stopRoller() {
        // Check if the button is pressed
        // Button is pressed
        this.roller.setPower(0);

    }

    }



