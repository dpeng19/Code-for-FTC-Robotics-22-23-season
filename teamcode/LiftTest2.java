package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous(name = "LiftTest2")
public class LiftTest2 extends LinearOpMode {
    public static double liftPos = 0;
    public DcMotorEx slide;
    //define PIDF variables and coefficients
    public static double kP = 0.0032, kI = 0.0001, kD = 0;
    public static double kV = 0, kA = 0, kStatic = 0;
    public static double kG = 0;
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) -> kG);
    @Override
    public void runOpMode(){
        slide = hardwareMap.get(DcMotorEx.class, "lift");
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        if(isStopRequested())return;
        while(!isStopRequested()){
            controller.setTargetPosition(liftPos);
            slide.setPower(controller.update(slide.getCurrentPosition()));
            telemetry.addData("measuredPos", slide.getCurrentPosition());
            telemetry.addData("targetPos", controller.getTargetPosition());
            //telemetry.addData("time", timer.seconds());
            telemetry.update();
        }
    }

}
