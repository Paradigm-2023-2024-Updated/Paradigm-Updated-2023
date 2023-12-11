package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareClasses.DriveTrain;

@Autonomous(name = "BadCompAuto")
public class BadCompAuto extends LinearOpMode {

    DriveTrain drivetrain = new DriveTrain();

    double time1, power, time2, time3;

    @Override
    public void runOpMode() throws InterruptedException {

        power = .9;
        time1 = 3;
        time2 = 6;
        time3 = 9;

        while(getRuntime() < time1) {
            drivetrain.driveFieldCentric(0, power, 0);
        }
        while((time1 < getRuntime()) && (getRuntime() < time2)) {
            drivetrain.driveFieldCentric(0, 0, -power);
        }
        while((time2 < getRuntime()) && (getRuntime() < time3)) {
            drivetrain.driveFieldCentric(power, 0, 0);
        }
        if (getRuntime() > time3) {
            drivetrain.driveFieldCentric(0, 0, 0);
        }

    }
}
