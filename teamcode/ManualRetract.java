package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ManualRetract")
public class ManualRetract extends LinearOpMode {
    private DcMotorEx slide;
    @Override
    public void runOpMode(){
        slide = hardwareMap.get(DcMotorEx.class, "lift");
        waitForStart();
        while(opModeIsActive()){
            slide.setPower(-gamepad1.left_stick_y * 0.5);
            telemetry.addData("pow", -gamepad1.left_stick_y * 0.5);
            telemetry.update();
        }
    }
}
