package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "WebCamCode")
public class WebCamCode extends OpMode {
    //CERTAIN CODE MIGHT BE IN THE WRONG SECTION! None of the good sources used the same "init, loop" format we use


    //establish webcam variable
    OpenCvWebcam webcam;
    @Override
    public void init(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new WebCamCode.examplePipeline());

        //Not sure if the timeout is necessary
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {

                webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //This code is meant to start streaming to phones
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Waiting for start");
        telemetry.update();

    }
    public void loop(){

        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();


    }

    class examplePipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline Running");

            Rect leftRect = new Rect(1,1,319,359);
            Rect rightRect = new Rect(320,1,319,359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut,rightRect,rectColor,2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop,leftCrop,2);
            Core.extractChannel(rightCrop,rightCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > rightavgfin){

                telemetry.addLine("Left");

            }
            else{

                telemetry.addLine("Right");
            }



            return (outPut);


        }
    }
}
