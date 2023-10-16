package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@TeleOp(name = "IMUAxisDebugger")
public class IMUAxisDebugger extends LinearOpMode {
    private BNO055IMU imu;
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocityX = 0.0;
    private double maxAngVelocityY = 0.0;
    private double maxAngVelocityZ = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        waitForStart();
        //drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        if(isStopRequested()) return;

        while (opModeIsActive()) {

            Orientation orientation =
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            AngularVelocity angularVelocity = imu.getAngularVelocity();

            telemetry.addData("Orientation Z Deg", "%.3f", orientation.firstAngle);
            telemetry.addData("Orientation Y Deg", "%.3f", orientation.secondAngle);
            telemetry.addData("Orientation X Deg", "%.3f", orientation.thirdAngle);
            //maxAngVelocity = Math.max(maxAngVelocity, Math.abs(angularVelocity.yRotationRate));
           // maxAngVelocityX = Math.max(maxAngVelocityX, Math.abs(angularVelocity.xRotationRate));
           // maxAngVelocityY = Math.max(maxAngVelocityY, Math.abs(angularVelocity.yRotationRate));
           // maxAngVelocityZ = Math.max(maxAngVelocityZ, Math.abs(angularVelocity.zRotationRate));
            telemetry.addData("Angular Velocity X Deg/s", "%.3f", angularVelocity.xRotationRate);
            telemetry.addData("Angular Velocity Y Deg/s", "%.3f", angularVelocity.yRotationRate);
            telemetry.addData("Angular Velocity Z Deg/s", "%.3f", angularVelocity.zRotationRate);

            telemetry.update();
        }
        //drive.setDrivePower(new Pose2d());

        //telemetry.addData("Max Angular Velocity X (deg)", Math.toDegrees(maxAngVelocityX));
        //telemetry.addData("Max Angular Velocity Y (deg)", Math.toDegrees(maxAngVelocityY));
        //telemetry.addData("Max Angular Velocity Z (deg)", Math.toDegrees(maxAngVelocityZ));
        //telemetry.addData("Max Angular Velocity X (deg)", maxAngVelocityX);
        telemetry.addData("Max Angular Velocity X (deg)", maxAngVelocityX);
        telemetry.addData("Max Angular Velocity Y (deg)", maxAngVelocityY);
        telemetry.addData("Max Angular Velocity Z (deg)", maxAngVelocityZ);
        //telemetry.addData("Max Angular Velocity Z (deg)", maxAngVelocityZ);
        telemetry.update();
    }
}
