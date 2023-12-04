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
            if (gamepad1.b) {
                // Button is pressed
                if (this.isMotorOn) {
                    // Stop the motor if it's currently spinning
                    this.roller.setPower(0);
                    isMotorOn = false;
                } else {
                    // Start the motor if it's currently stopped
                    this.roller.setPower(1.0); // You can adjust the power level as needed
                    isMotorOn = true;
                }

            }

        }

    }



