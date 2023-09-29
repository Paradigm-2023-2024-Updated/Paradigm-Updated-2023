
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;


public class OpenCVTest extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class,"webcam");
        int cameraMonitorViewId  = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","webCamName","webCamName");



    }

    @Override
    public void loop() {

    }


}

