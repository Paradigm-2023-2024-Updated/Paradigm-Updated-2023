package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "AprilTagTestCode")
public class AprilTagTestCode extends OpMode {



    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .build();

    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640,480)) //adjust width and height as needed
            .build();

    @Override
    public void init(){
   // public void runOpMode() throws InterruptedException{



    }

    @Override
    public void loop(){
   // waitForStart();

    //while (!isStopRequested() && opModeIsActive()){

        if (tagProcessor.getDetections().size()>0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            telemetry.addData("x",tag.ftcPose.x);
            telemetry.addData("y",tag.ftcPose.y);
            telemetry.addData("z",tag.ftcPose.z);
            telemetry.addData("roll",tag.ftcPose.roll);
            telemetry.addData("pitch",tag.ftcPose.pitch);
            telemetry.addData("yaw",tag.ftcPose.yaw);
        }

        telemetry.update();
    }


}
