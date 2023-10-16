package org.firstinspires.ftc.teamcode;

import android.icu.text.RelativeDateTimeFormatter;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.ThermalEquilibrium.homeostasis.Filters.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "ColorDistanceTest")
public class ColorDistanceTest extends LinearOpMode {
    //DistanceSensor rightDist, centerDist, leftDist;
    DistanceSensor dist1, dist2, dist3;
    //ColorSensor color;
    public void runOpMode(){
        double Q = 1; // High values put more emphasis on the sensor.
        double R = 0.7; // High Values put more emphasis on regression.
        int N = 3; // The number of estimates in the past we perform regression on.
        KalmanFilter filter = new KalmanFilter(Q,R,N);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dist1 = hardwareMap.get(DistanceSensor.class, "distColor1");
        dist2 = hardwareMap.get(DistanceSensor.class, "distColor2");
        dist3 = hardwareMap.get(DistanceSensor.class, "distColor3");
        //dist2 = hardwareMap.get(DistanceSensor.class, "distColor2");
        //color = hardwareMap.get(ColorSensor.class, "distColor");

        /*
        rightDist = hardwareMap.get(DistanceSensor.class, "distColor1");
        centerDist = hardwareMap.get(DistanceSensor.class, "distColor2");
        leftDist = hardwareMap.get(DistanceSensor.class, "distColor3");

         */
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
         //color = hardwareMap.get(ColorSensor.class, "distColor");

        waitForStart();
        while(opModeIsActive()){
            /*
            double currentValue1 = dist1.getDistance(DistanceUnit.INCH);  // imaginary, noisy sensor
            double estimate1 = filter.estimate(currentValue1); // smoothed sensor
            double currentValue2 = dist2.getDistance(DistanceUnit.INCH);  // imaginary, noisy sensor
            double estimate2 = filter.estimate(currentValue2); // smoothed sensor
            double currentValue3 = dist3.getDistance(DistanceUnit.INCH);  // imaginary, noisy sensor
            double estimate3 = filter.estimate(currentValue3); // smoothed sensor
            telemetry.addData("dist1Noisy1", currentValue1);
            telemetry.addData("smoothed1", estimate1);
            telemetry.addData("dist1Noisy2", currentValue2);
            telemetry.addData("smoothed2", estimate2);
            telemetry.addData("dist1Noisy3", currentValue3);
            telemetry.addData("smoothed3", estimate3);

             */
            /*
            String telem;
            double RD = rightDist.getDistance(DistanceUnit.INCH);
            double CD = centerDist.getDistance(DistanceUnit.INCH);
            double LD = leftDist.getDistance(DistanceUnit.INCH);
            if(Math.abs(Math.min(CD, Math.min(RD, LD)) - CD) < 0.01 && RD - CD > 2 && LD - CD > 2)
                telem = "Centered";
            else if(Math.abs(RD - CD) <= 2 && Math.abs(Math.max(CD, Math.max(RD, LD)) - LD) < 0.01 )
                telem = "Move Right Slightly";
            else if(Math.abs(Math.min(CD, Math.min(RD, LD)) - RD) < 0.01 && CD - RD > 2 && LD - RD > 2)
                telem = "Move Right More";

            else if(Math.abs(LD - CD) <= 2)
                telem = "Move Left Slightly";
            else if(Math.abs(Math.min(CD, Math.min(RD, LD)) - LD) < 0.01 && CD - LD > 2 && RD - LD > 2)
                telem = "Move Left More";

            else
                telem = "NONE";
            //Math.abs(Math.max(CD, Math.max(RD, LD)) - LD) < 0.01
            telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("centerDist", centerDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("leftDist", leftDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("action", telem);
            /*
             telemetry.addData("red", color.red());
             telemetry.addData("green", color.green());
            telemetry.addData("blue", color.blue());

             */
            /*
            telemetry.addData("distance", dist.getDistance(DistanceUnit.CM));
            telemetry.addData("red", color.red());
            telemetry.addData("green", color.green());
            telemetry.addData("blue", color.blue());
            telemetry.addData("color", ColorRanges.GetColor(color.red(), color.green(), color.blue()));

             */
            //telemetry.addData("hue", ColorRanges.getHue());
            telemetry.addData("distance1", filter.estimate(dist1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("distance2", filter.estimate(dist2.getDistance(DistanceUnit.INCH)));
            telemetry.addData("distance3", filter.estimate(dist3.getDistance(DistanceUnit.INCH)));
            telemetry.update();

        }
    }
}
