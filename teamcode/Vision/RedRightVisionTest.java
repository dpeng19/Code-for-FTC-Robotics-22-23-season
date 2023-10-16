package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedRightVisionTest")
public class RedRightVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Scanner scanner = new Scanner(hardwareMap, telemetry, false);
        waitForStart();
        Color color = scanner.getResult();
        telemetry.addData("color", color);
        telemetry.update();

        scanner.stop();
        /*
        while(opModeIsActive() && !isStopRequested()){
            Color color = scanner.getResult();
            telemetry.addData("color", color.toString());
            //scanner.getValues();
            telemetry.update();
        }

         */

        //scanner.stop();
    }
}