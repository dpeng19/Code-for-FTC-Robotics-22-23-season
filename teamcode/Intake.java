package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
@Disabled
@TeleOp(name = "Intake")
public class Intake extends LinearOpMode {
    private DcMotorEx motor1, motor2;
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        MotorConfigurationType motorConfigurationType = motor1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor1.setMotorType(motorConfigurationType);
        MotorConfigurationType motorConfigurationType2 = motor2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        motor2.setMotorType(motorConfigurationType2);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()){
            motor1.setVelocity(1600);
            motor2.setVelocity(1600);
            telemetry.addData("motor1vel", motor1.getVelocity());
            telemetry.addData("motor2vel", motor2.getVelocity());
            telemetry.update();

        }

    }
}
