package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "ColSensor")
public class ColSensor extends OpMode {
    //establishing color sensor variable
    ColorSensor sensorColor;
    @Override
    public void init(){
        //Connect color sensor variable to hardware
        sensorColor = hardwareMap.get(ColorSensor.class, "Color");

    }

    @Override
    public void loop(){

        //Display color values on phone
        telemetry.addData("Red", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue", sensorColor.blue());
        telemetry.update();
    }

}
