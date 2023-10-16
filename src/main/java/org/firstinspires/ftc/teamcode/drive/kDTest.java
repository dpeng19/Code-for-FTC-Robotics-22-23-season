package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.CriticallyDampedPD.solveKD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.TRANSLATIONAL_PID;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.CriticallyDampedPD;
@Disabled
@TeleOp(name = "kDTest")
public class kDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("kp", TRANSLATIONAL_PID.kP);
            telemetry.addData("kV", kV * 12/batteryVoltageSensor.getVoltage());
            telemetry.addData("kA", kA * 12/batteryVoltageSensor.getVoltage());
            telemetry.addData("kD", solveKD(TRANSLATIONAL_PID.kP, kV * 12/batteryVoltageSensor.getVoltage() , kA * 12/batteryVoltageSensor.getVoltage()));
            telemetry.update();
        }
    }
}
