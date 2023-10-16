package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
@Autonomous(name = "PIDTurnTest")
public class PIDTurnTest extends LinearOpMode {
    private DcMotorEx FR, FL, BR, BL;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private final double TICKS_PER_REV = 537.7;
    private final double radius = 1.89;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.get(DcMotorEx.class, "FrontRight");
        FL = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        BR = hardwareMap.get(DcMotorEx.class, "BackRight");
        BL = hardwareMap.get(DcMotorEx.class, "BackLeft");

        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        initializeIMU();
        
        FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        



        waitForStart();

        /*
        turn(90);
        sleep(3000);
        turnTo(-90);

         */
        FR.setPower(0.4);
        FL.setPower(0.4);
        BR.setPower(0.4);
        BL.setPower(0.4);
        sleep(3000);
        while(BR.isBusy()){
            telemetry.addData("FRVel", FR.getVelocity());
            telemetry.addData("FRVel", FR.getVelocity());
            telemetry.addData("FRVel", FR.getVelocity());
            telemetry.addData("FRVel", FR.getVelocity());
            telemetry.update();
        }



        //move(34, 50);


        //turnPID(90);


    }
    public void initializeIMU()
    {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle()
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if(deltaAngle > 180)
            deltaAngle -= 360;
        else if(deltaAngle < -180)
            deltaAngle += 360;
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees)
    {
        resetAngle();
        double error = degrees;
        while(opModeIsActive() && Math.abs(error) > 2)
        {
            double motorPower = (error < 0 ? -0.3: 0.3);
            FR.setPower(motorPower);
            FL.setPower(-motorPower);
            BR.setPower(motorPower);
            BL.setPower(-motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

    }
    public void turnTo(double degrees)
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        if(error > 180)
            error -= 360;
        else if(error < -180)
            error += 360;
        turn(error);
    }
    public double getAbsoluteAngle()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void turnToPID(double targetAngle)
    {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        while(opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1)
        {
            double motorPower = pid.update(getAbsoluteAngle());
            FR.setPower(motorPower);
            FL.setPower(-motorPower);
            BR.setPower(motorPower);
            BL.setPower(-motorPower);
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    void turnPID(double degrees)
    {
        turnToPID(degrees + getAbsoluteAngle());
    }
    private void move(double inches, double max_in_per_s){
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  //works
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // ticks = in * rev / in * ticks / rev
        int ticks = (int)(inches * (1 / (2 * Math.PI * radius)) * TICKS_PER_REV);

        FR.setTargetPosition(ticks);
        FL.setTargetPosition(ticks);
        BR.setTargetPosition(ticks);
        BL.setTargetPosition(ticks);

        FR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        // ticks / s = in / s * rev / in * ticks / rev
        double angularRate = max_in_per_s * (1 / (2 * Math.PI * radius)) * TICKS_PER_REV;



        FR.setVelocity(angularRate);
        FL.setVelocity(angularRate);
        BR.setVelocity(angularRate);
        BL.setVelocity(angularRate);




        while(BL.isBusy() && BR.isBusy())
        {
            telemetry.addData("frontR", FR.getCurrentPosition());
            telemetry.addData("frontL", FL.getCurrentPosition());
            telemetry.addData("backR", BR.getCurrentPosition());
            telemetry.addData("backL", BL.getCurrentPosition());
            telemetry.update();
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);




    }

}






