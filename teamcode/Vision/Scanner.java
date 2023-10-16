package org.firstinspires.ftc.teamcode.Vision;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.CountDownLatch;


public class Scanner extends OpenCvPipeline {

    Mat GreenMat = new Mat();
    Mat RedMat = new Mat();
    Mat YellowMat = new Mat();
    Mat GreenSub= new Mat();
    Mat YellowSub = new Mat();
    Mat RedSub = new Mat();
    Rect ROI =new Rect(new Point(320/3.0 + 37, 85), new Point(2 * 320 / 3.0 - 35, 150));
    private Color color = null;
    private Telemetry telemetry;
    private OpenCvCamera webcam;
    private final CountDownLatch latch = new CountDownLatch(1);


    public Scanner(@NonNull HardwareMap hardwareMap, Telemetry t, boolean isRight){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if(isRight)
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rightWebcam"), cameraMonitorViewId);
        else
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "leftWebcam"), cameraMonitorViewId);
        webcam.setPipeline(this);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, GreenMat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(GreenMat, GreenMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, RedMat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(RedMat, RedMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, YellowMat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(YellowMat, YellowMat, Imgproc.COLOR_RGB2HSV);
        //Green
        //H: 70, 170 :: 70/2, 170/2
        //S: 100 255
        //V: 100 255
        Scalar GreenLowBound = new Scalar(81.0/2, 0, 0);
        Scalar GreenUpBound = new Scalar(170.0/2, 255, 255);
        //Red
        //H: 197, 320 :: 197/2, 320/2
        //S: 100 255
        //V: 100 255
        Scalar RedLowBound = new Scalar(270.0/2, 0, 0);
        Scalar RedUpBound = new Scalar(360.0/2, 255, 255);
        //Yellow
        //H: 5, 60 :: 5/2, 60/2
        //S: 100 255
        //V: 100 255
        Scalar YellowLowBound = new Scalar(35.0/2, 0, 0);
        Scalar YellowUpBound = new Scalar(80.0/2, 255, 255);
        Core.inRange(GreenMat, GreenLowBound, GreenUpBound, GreenMat);
        Core.inRange(RedMat, RedLowBound, RedUpBound, RedMat);
        Core.inRange(YellowMat, YellowLowBound, YellowUpBound, YellowMat);
        GreenSub = GreenMat.submat(ROI);
        YellowSub = YellowMat.submat(ROI);
        RedSub = RedMat.submat(ROI);
        double YellowVal = Core.sumElems(YellowSub).val[0];
        double GreenVal = Core.sumElems(GreenSub).val[0];
        double RedVal = Core.sumElems(RedSub).val[0];

        GreenSub.release();
        YellowSub.release();
        RedSub.release();
        GreenMat.release();
        YellowMat.release();
        RedMat.release();



        double maxVal = Math.max(YellowVal, Math.max(GreenVal,RedVal));
        if(maxVal == YellowVal) {
            color = Color.YELLOW;
        }
        else if(maxVal == GreenVal){
            color = Color.GREEN;
        }
        else
            color = Color.RED;
        latch.countDown();


        Scalar scal = new Scalar(255, 0, 0);
        Imgproc.rectangle(input, ROI, scal);


        //telemetry.addData("Color", color.toString().toLowerCase());
        //telemetry.update();

        return input;
    }
    public Color getResult() throws InterruptedException{
        //implement countdown latch
        if(color == null){
            latch.await();
        }
        return color;
    }
    public void stop(){
        webcam.stopStreaming();
    }
    public void getValues(){
        //telemetry.addData("maxVal", maxVal);
        /*
        telemetry.addData("Yellow", YellowVal);
        telemetry.addData("Red", RedVal);
        telemetry.addData("green", GreenVal);

         */
        //telemetry.update();
    }

}
