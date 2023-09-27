package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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

        //Color.RGBToHSV(sensorColor.red() * 8, sensorColor.green() * 8, sensorColor.blue() * 8, sensorColor.alpha() * 8);

        //Display color values on phone
        telemetry.addData("Red", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue", sensorColor.blue());
        telemetry.update();
    }

    // Function trying to convert RGB to HSV
    static void RGBtoHSV(double r, double g, double b){

        //divide RGB by 255 so instead of being from 0 - 255 they are 0 - 1
        r = r / 255.0;
        g = g /255.0;
        b = b /255.0;

        double cmax = Math.max(r, Math.max(g,b)); //maximum of R
        double cmin = Math.min(r, Math.min(r, Math.min(g, b))); //minimum of R
        double diff = cmax - cmin; //difference of cmaximum and cminimum
        double h = -1, s = -1;


        // CALCULATING HUE!
        //if cmaximum and cminimum are equal then h = 0
        if (cmax == cmin) {
            h = 0;
        }

        // if cmax == r then calculate h
        else if (cmax == r) {
            h = (60 * ((g - b) / diff) + 30) % 360;
        }

        //if cmax equals g then calculate hue
        else if (cmax == g){
            h = (60 * ((b - r) / diff) + 120) % 360;
        }
        //if cmax equals B then calculate hue
        else if (cmax == b){
            h = (60 * ((r - g) / diff) + 240) % 360;
        }

        //IF cmax equals 0 then saturation is 0
        else if (cmax == 0){
            s = 0;
        }
        else{
            s = (diff / cmax) * 100;
        }

        //compute value
        double v = cmax * 100;
        System.out.println("(" + h + " " + s + " " + v + ")");
}}
