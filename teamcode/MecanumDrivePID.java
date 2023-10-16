package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//Improvement from MecanumDrive
//1. limited movement to forward, backward, straferight, strafeleft to help with gamepad sensitivity
//2. Uses PID to correct heading of the robot, <=4 degrees of heading error at any moment in time
//Thus, the movement is much straighter
//Problem is robot's movement is not very smooth and turning sucks (huge delay in gamepad input)
@Disabled
@TeleOp(name = "MecanumDrivePID", group = "TeleOp")
public class MecanumDrivePID extends LinearOpMode {
    /*
    public enum GamepadState {
        IDLE,
        FOLLOW,
        PID_ADJUST
    }

     */



    //GamepadState gamepadState = GamepadState.IDLE;
    boolean isSimpleMode = true;
    private DcMotor FR, FL, BR, BL;
    private static final double pwr_Mult = 0.6;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private boolean isLastTurn = true;

    ElapsedTime timer = new ElapsedTime();
    private double derivative = 0;
    private double integralSum = 0;
    private double lastTime = 0;
    private double lastError = 0;
    public static double p = 0.017, i = 0, d = 0.05;
    private double targetHead;

    @Override
    public void runOpMode() {
        FR = hardwareMap.get(DcMotor.class, "FrontRight");
        FL = hardwareMap.get(DcMotor.class, "FrontLeft");
        BR = hardwareMap.get(DcMotor.class, "BackRight");
        BL = hardwareMap.get(DcMotor.class, "BackLeft");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        initializeIMU();


        waitForStart();

        if (isStopRequested()) return;
        //resetAngle();
        while (opModeIsActive()) {

            if(gamepad1.right_stick_x == 0 && isLastTurn){
                targetHead = getAngle();
                timer.reset();
                derivative = 0;
                integralSum = 0;
                lastTime = 0;
                lastError = 0;
                isLastTurn = false;
            }

            //controls x, y, rot in that order
            setPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, pwr_Mult, isSimpleMode, getAngle(), targetHead);
            isLastTurn = (gamepad1.right_stick_x != 0);
            /*
            telemetry.addData("FRPower", FR.getPower());
            telemetry.addData("FLPower", FL.getPower());
            telemetry.addData("BRPower", BR.getPower());
            telemetry.addData("BLPower", BL.getPower());
            telemetry.addData("gyro", getAngle());

             */
            telemetry.addData("Heading", getAngle());
            telemetry.addData("Target", targetHead);
            telemetry.update();

        }
    }

    public void setPower(double x, double y, double rot, double pwr_Mult, boolean isSimpleMode, double heading, double target) {
        //give drivers more error with gamepad sticks
        if (isSimpleMode) {
            if (x != 0 && y != 0) {
                double angle = Math.atan2(-y, x);
                angle = Math.toDegrees(angle);
                int angleI = (int) (Math.round(angle));
                if (angleI > 45 && angleI <= 135) {

                    x = 0;
                    y = -1;
                } else if ((angleI > 135 && angleI <= 180) || (angleI > -180 && angleI <= -135)) {
                    x = -1;
                    y = 0;
                } else if (angleI > -135 && angleI <= -45) {
                    x = 0;
                    y = 1;
                } else if ((angleI >= 0 && angleI <= 45) || (angleI > -45 && angleI < 0)) {
                    x = 1; //maybe change instead of 0 and 1s with x and ys
                    y = 0;
                }
            }


        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1); //maybe replace this with alternate
        double FRPower = (y - x - rot) / denominator * pwr_Mult;
        double FLPower = (y + x + rot) / denominator * pwr_Mult;
        double BRPower = (y + x - rot) / denominator * pwr_Mult;
        double BLPower = (y - x + rot) / denominator * pwr_Mult;
        //alternatively
        /*
        double denominator = Math.max(Math.max(Math.abs(FRPower), Math.abs(BLPower)), Math.max(Math.abs(FLPower), Math.abs(BRPower)));
        */
        double normalize = 1;
        double correction = 0;
        if(rot == 0){
            double error = target - heading;
            if(lastTime > 0)
                derivative = (error - lastError) / (timer.milliseconds() - lastTime);
            integralSum *= Math.signum(error);
            integralSum += error;

            if(Math.abs(heading - target) != 0)
                correction = (p * error) + (i * integralSum) + (d * derivative);
            else
                integralSum = 0;

            double dividePID = Math.max(Math.max(Math.abs(FRPower + correction), Math.abs(BLPower - correction)),
                    Math.max(Math.abs(FLPower - correction), Math.abs(BRPower + correction)));
            normalize = Math.max(dividePID, 1);



            lastError = error;
            lastTime = timer.milliseconds();
        }
        FR.setPower((FRPower + correction) / normalize);
        FL.setPower((FLPower - correction) / normalize);
        BR.setPower((BRPower + correction) / normalize);
        BL.setPower((BLPower - correction) / normalize);




    }

    public void initializeIMU() {

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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaAngle > 180)
            deltaAngle -= 360;
        else if (deltaAngle < -180)
            deltaAngle += 360;
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
}
