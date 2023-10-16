package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;
@Disabled
@Autonomous (name = "DebugAngularVelocity")
public class DebugAngularVelocity extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocityX = 0.0;
    private double maxAngVelocityY = 0.0;
    private double maxAngVelocityZ = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            //Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            AngularVelocity ang = drive.imu.getAngularVelocity();
            maxAngVelocityX = Math.max(maxAngVelocityX, Math.abs(Math.toDegrees(ang.xRotationRate)));
            maxAngVelocityY = Math.max(maxAngVelocityY, Math.abs(Math.toDegrees(ang.yRotationRate)));
            maxAngVelocityZ = Math.max(maxAngVelocityZ, Math.abs(Math.toDegrees(ang.zRotationRate)));
            //maxAngVelocity = Math.max(maxAngVelocity, Math.toDegrees(drive.getExternalHeadingVelocity()));

        }
        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity X (deg)", maxAngVelocityX);
        telemetry.addData("Max Angular Velocity Y (deg)", maxAngVelocityY);
        telemetry.addData("Max Angular Velocity Z (deg)", maxAngVelocityZ);
        //telemetry.addData("Max Angular Velocity (deg)", maxAngVelocity);
        telemetry.update();
    }
}
