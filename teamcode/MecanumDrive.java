
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {
    public enum GamepadState{
        IDLE,
        FOLLOW,
        PID_ADJUST
    }
    GamepadState gamepadState = GamepadState.IDLE;
    boolean isSimpleMode = false;
    private DcMotor FR,FL,BR,BL;
    private static final double pwr_Mult = 0.6;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
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
        while(opModeIsActive())
        {
            /*
            switch(gamepadState){
                case IDLE:
                    if(!(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)){
                        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, pwr_Mult);
                        gamepadState = GamepadState.FOLLOW;
                    }
                    break;
                case FOLLOW:
                    if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0)
                        gamepadState = GamepadState.IDLE;
                    //if turn button is not pressed, keep track of heading

                    //write code for if not turn and heading is off by > 0.5 degrees, then
                    //transition to PID_ADJUST state and run PID
                    break;
                case PID_ADJUST:
                    //write code for if x, y, rot all equal 0, transition to FOLLOW state

                    //if heading error <= 0.5 degrees or if turn is pressed, transition to FOLLOW state
                    break;
            }
            */

            //controls x, y, rot in that order
            setPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, pwr_Mult, isSimpleMode);

            if(gamepad1.x)
                isSimpleMode = !isSimpleMode;
            telemetry.addData("FRPower", FR.getPower());
            telemetry.addData("FLPower", FL.getPower());
            telemetry.addData("BRPower", BR.getPower());
            telemetry.addData("BLPower", BL.getPower());
            telemetry.addData("gyro", getAngle());
            telemetry.update();

        }
    }
    public void setPower(double x, double y, double rot, double pwr_Mult, boolean isSimpleMode)
    {
        //give drivers more error with gamepad sticks
        if(isSimpleMode){
            if(x!= 0 && y != 0) {
                double angle = Math.atan2(-y, x);
                angle = Math.toDegrees(angle);
                int angleI = (int) (Math.round(angle));
                if (angleI > 45 && angleI <= 135) {

                    x = 0;
                    y = -1;
                }
                else if((angleI > 135 && angleI <= 180) || (angleI > -180 && angleI <= -135)){
                    x = -1;
                    y = 0;
                }
                else if(angleI > -135 && angleI <= -45){
                    x = 0;
                    y = 1;
                }
                else if((angleI >= 0  && angleI <= 45) || (angleI > -45 && angleI < 0)){
                    x = 1; //maybe change instead of 0 and 1s with x and ys
                    y = 0;
                }
            }


        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1); //maybe replace this with alternate
        double FRPower = (y - x - rot) / denominator;
        double FLPower = (y + x + rot) / denominator;
        double BRPower = (y + x - rot) / denominator;
        double BLPower = (y - x + rot) / denominator;
        //alternatively
        /*
        double denominator = Math.max(Math.max(Math.abs(FRPower), Math.abs(BLPower)), Math.max(Math.abs(FLPower), Math.abs(BRPower)));
        */


        FR.setPower(pwr_Mult * FRPower);
        FL.setPower(pwr_Mult * FLPower);
        BR.setPower(pwr_Mult * BRPower);
        BL.setPower(pwr_Mult * BLPower);




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
            double motorPower = (error < 0 ? 0.3: -0.3);
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
            FR.setPower(-motorPower);
            FL.setPower(motorPower);
            BR.setPower(-motorPower);
            BL.setPower(motorPower);
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    public void turnPID(double degrees)
    {
        turnToPID(degrees + getAbsoluteAngle());
    }
    public void CorrectPID(double x, double y){
        double angle = Math.atan2(y, x);
        angle = Math.toDegrees(angle);
        int target;
        if(angle >= 0)
            target = (int)(Math.round(angle - 90.0));
        else{

        }

    }
}

