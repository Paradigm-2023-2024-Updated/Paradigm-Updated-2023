package org.firstinspires.ftc.teamcode.hardwareClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    Servo launcher;

    public Launcher() {

    }

    public void init(HardwareMap hardwareMap) {

        this.launcher = hardwareMap.servo.get("launcher");

    }

    public void launchAirplane() {

        this.launcher.setPosition(1);

    }


}

