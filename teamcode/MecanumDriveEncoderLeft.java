package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
@Disabled
@Config
@TeleOp(name = "MecanumDriveEncoderLeft", group = "TeleOp")
public class MecanumDriveEncoderLeft extends LinearOpMode {

    //this mode limits robot movement to forwards, backwards, straferight, strafeleft
    //helps with gamepad sensitivity
    boolean isSimpleMode = true;

    //stores powers for the motors (y = -1 -> forward, y = 1 -> backward, x = 1 -> right, x = -1 -> left)
    private int[][] dir = {{0, 1},{-1, 0},{0, -1}, {1, 0}};

    private DcMotorEx FR, FL, BR, BL;
    public static double pwr_Mult = 0.6; //speed of robot, ranges from -1 to 1, adjust closer to 1 for faster speed

    //initialize IMU variables
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private boolean isLastTurn = true;
    private boolean isFirstRun = true;
    private boolean pGA1RX = false;
    private boolean checkStop = false;
    private boolean pGA1X = false;
    private boolean pGA1Y = false;
    //private boolean isXPressed = false;
    //private boolean isYPressed = false;
    //private boolean isFirstCloseClaw = false;
    private boolean pGA1RB = false;
    private boolean clawOpen = true;
    private boolean needLift = false;
    private boolean pGA1A = false;
    private boolean pGA1B = false;
    //private boolean pGA1RB = false;



    //robot orientation (0 = 0 degrees, 1 = -90 degrees, 2 = 180/-180 degrees, 3 = 90 degrees)
    //initially 0 because robot starts at initial orientation of 0 degrees (facing forwards)
    private int Heading_State = 0;

    //variables for PID Correction calculations to keep robot straight
    ElapsedTime timer = new ElapsedTime();
    private double derivative = 0;
    private double integralSum = 0;
    private double lastTime = 0;
    private double lastError = 0;
    public static double p = 0.015, i = 0, d = 0.05;
    private double targetHead = 0;

    private double lastLiftPos = 0;
    //may have to adjust
    private final double LOW_TARGET = 1800;
    private final double MED_TARGET = 3200;
    private final double HIGH_TARGET = 4200;


    private final double STACK_TIME = 1.0;
    //ElapsedTime StackTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    private int debug = 0;

    enum LiftState {
        START,
        EXTEND,
        STACK,
        RETRACT
    }
    LiftState liftState = LiftState.START;
    @Override
    public void runOpMode() {

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        FR = hardwareMap.get(DcMotorEx.class, "FrontRight");
        FL = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        BR = hardwareMap.get(DcMotorEx.class, "BackRight");
        BL = hardwareMap.get(DcMotorEx.class, "BackLeft");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        //RUN_USING_ENCODER mode helps motors maintain equal speeds to each other
        FR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        initializeIMU();

        timer.reset();
        waitForStart();

        //FTC dashboard for some cool graphs and PID tuning
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;
        //claw.close();
        //lift.setTarget(0);
        //resetAngle();
        pwr_Mult = 0.6;
        targetHead = 0;
        //imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle = 90;
        while (opModeIsActive()) {



/*
            if(isXPressed && !gamepad1.x)
                isXPressed = false;
            if(isYPressed && !gamepad1.y)
                isYPressed = false;

 */

            //robot has been turning but now is not turning
            //if this is the case, set targetHead to current angle so the robot knows to keep this angle
            //reset PID variables
            //sets power of motors
            setPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, pwr_Mult, isSimpleMode, getAbsoluteAngle(), targetHead);
            boolean ga1RX = (gamepad1.right_stick_x != 0);
            if(pGA1RX && !ga1RX ){
                //if(gamepad1.right_stick_x == 0 && isLastTurn){
                //if(!isFirstRun) {
                checkStop = true;
                /*
                    targetHead = getAbsoluteAngle();
                    timer.reset();
                    derivative = 0;
                    integralSum = 0;
                    lastTime = 0;
                    lastError = 0;

                 */
                //}
                /*
                else
                    isFirstRun = false;
                isLastTurn = false;
                 */
            }
            pGA1RX = ga1RX;

            if(checkStop){
                if(FR.getVelocity() <= 0.1 && FL.getVelocity() <= 0.1 && BR.getVelocity() <= 0.1 && BL.getVelocity() <= 0.1){
                    targetHead = getAbsoluteAngle();
                    resetPID();
                    checkStop = false;
                }
            }
            boolean ga1X = gamepad1.x;
            if(ga1X && !pGA1X){
                if(Math.abs(targetHead) != 180)
                    targetHead -= 90;
                else
                    targetHead = 90;
                resetPID();
                //isXPressed = true;

            }
            pGA1X = ga1X;
            boolean ga1Y = gamepad1.y;
            if(ga1Y && !pGA1Y){
                if(Math.abs(targetHead) != 180)
                    targetHead += 90;
                else
                    targetHead = -90;
                resetPID();
                //isYPressed = true;
            }
            pGA1Y = ga1Y;
            boolean ga1A = gamepad1.a;
            if(ga1A && !pGA1A){
                if(pwr_Mult == 0.6)
                    pwr_Mult = 0.25;
                else
                    pwr_Mult = 0.6;
            }
            pGA1A = ga1A;
            boolean ga1B = gamepad1.b;
            if(ga1B && !pGA1B){
                targetHead = 0;//maybe change to closest 90, 180, -90, 0 etc.
                resetPID();
            }
            pGA1B = ga1B;
/*
            boolean ga1RB = gamepad1.right_bumper;
            if(ga1RB && !pGA1RB)
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle = 0;
            pGA1RB = ga1RB;

 */


            // isLastTurn = (gamepad1.right_stick_x != 0);

            telemetry.addData("liftState", liftState);
            telemetry.addData("x", gamepad2.x);
            telemetry.addData("y", gamepad2.y);
            telemetry.addData("b", gamepad2.b);
            //if(gamepad2.right_bumper)
            //claw.close();

            boolean ga1RB = gamepad1.right_bumper; //may want to change to gamepad1 so driver controls claw too, may be easier
            if(ga1RB && !pGA1RB){
                if(clawOpen){
                    claw.close();
                }
                else
                    claw.open();
                clawOpen = !clawOpen;
                needLift = true;
                liftTimer.reset();

            }
            pGA1RB = ga1RB;

            if(needLift && liftTimer.seconds() > 0.5){
                if(clawOpen) {
                    lift.setTarget(-10);//may have to adjust
                    targetHead = 180;
                    pwr_Mult = 0.25;
                }
                else{
                    lift.setTarget(250);
                    targetHead = 90;
                    pwr_Mult = 0.6;
                }
                needLift = false;

            }




            switch (liftState) {
                case START:
                    if(gamepad2.x) {
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                        //debug = 1;
                    }
                    if(gamepad2.y) {
                        lift.setTarget(MED_TARGET);
                        liftState = LiftState.EXTEND;
                        //debug = 2;
                    }
                    if(gamepad2.b){
                        lift.setTarget(HIGH_TARGET);
                        liftState = LiftState.EXTEND;
                        //debug = 3;
                    }

                    break;
                case EXTEND:
                    //need to tune pidf to hit setpoints better-100 to much error
                    if(Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition()  - lastLiftPos) <= 1) {
                        //StackTimer.reset();
                        pwr_Mult = 0.25;//maybe change this to faster
                        liftState = LiftState.STACK;
                    }
                    break;
                case STACK:
                    if(gamepad2.a){
                        claw.open();
                        clawOpen = true;
                        lift.setTarget(-30);//may have to adjust
                        liftState = LiftState.RETRACT;
                    }
                    break;
                case RETRACT:
                    if (Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 10 || Math.abs(lift.slide.getCurrentPosition()  - lastLiftPos) <= 1) {
                        liftState = LiftState.START;
                        //targetHead = -180; //may need to wait a bit, too violent
                        pwr_Mult = 0.6;

                    }
                    break;
            }
            lastLiftPos = lift.slide.getCurrentPosition();
            if (gamepad2.left_bumper && liftState != LiftState.START) {
                liftState = LiftState.START;
                lift.setTarget(0);
            }

            //telemetry.addData("debug", debug);
            lift.update();
            telemetry.addData("liftPosition", lift.slide.getCurrentPosition());
            telemetry.addData("liftTarget", lift.getTarget());
            telemetry.addData("Heading", getAbsoluteAngle());
            telemetry.addData("Target", targetHead);
            /*
            telemetry.addData("FRVel", FR.getVelocity());
            telemetry.addData("FLVel", FL.getVelocity());
            telemetry.addData("BRVel", BR.getVelocity());
            telemetry.addData("BLVel", BL.getVelocity());

             */
            /*
            telemetry.addData("FRPIDF", FR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("FLPIDF", FL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("BRPIDF", BR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("BLPIDF", BL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            */
            telemetry.addData("HeadingState", Heading_State);


            telemetry.update();

        }
    }

    public void setPower(double x, double y, double rot, double pwr_Mult, boolean isSimpleMode, double heading, double target) {
        //angle wrapping for optimal turns-to be worked on
        /*
        if(Math.abs(180 - Math.abs(target)) < 5 && target > 0 && heading < 0){
            heading += 360;
        }
        if(Math.abs(180 - Math.abs(target)) < 5 && target < 0 && heading > 0){
            heading -= 360;
        }

         */

        //telemetry.update();
        //keeps target between -180 to 180
        if (target > 180)
            target -= 360;
        else if (target  < -180)
            target += 360;



        //give drivers more error with gamepad sticks
        if(x == 0 && y == 0 && !gamepad1.x && !gamepad1.y){
            if(Math.abs(heading) < 5)
                Heading_State = 0;
            else if(Math.abs(-90 - heading) < 5)
                Heading_State = 1;
            else if(180 - Math.abs(heading) < 5)
                Heading_State = 2;
            else if(Math.abs(90 - heading) < 5)
                Heading_State = 3;
            /*
            if(target == 0 && Math.abs(target - heading) < 5)
                Heading_State = 0;
            else if(target == -90 && Math.abs(target - heading) < 5)
                Heading_State = 1;
            else if(Math.abs(target) == 180 && 180 - Math.abs(heading) < 5)
                Heading_State = 2;
            else if(target == 90 && Math.abs(target - heading) < 5)
                Heading_State = 3;

             */
        }
        telemetry.addData("xinit", x);
        telemetry.addData("yinit", y);
        if (isSimpleMode) {
            if (x != 0 || y != 0) {
                double angle = Math.atan2(y, x);
                angle = Math.toDegrees(angle);
                int angleI = (int) (Math.round(angle));
                telemetry.addData("angleI", angleI);

                //press up on left gamepad
                if (angleI > 45 && angleI <= 135) {
                    x = dir[Heading_State][0];
                    y = dir[Heading_State][1];

                }
                //press left on left gamepad
                else if ((angleI > 135 && angleI <= 180) || (angleI >= -180 && angleI <= -135)) {
                    x = dir[(Heading_State + 1) % 4][0];
                    y = dir[(Heading_State + 1) % 4][1];

                }
                //press down on left gamepad
                else if (angleI > -135 && angleI <= -45) {
                    x = dir[(Heading_State + 2) % 4][0];
                    y = dir[(Heading_State + 2) % 4][1];

                }
                //press right on left gamepad
                else if ((angleI >= 0 && angleI <= 45) || (angleI > -45 && angleI < 0)) {
                    //maybe change instead of 0 and 1s with x and ys
                    x = dir[(Heading_State + 3) % 4][0];
                    y = dir[(Heading_State + 3) % 4][1];

                }

            }
            telemetry.addData("x", x);
            telemetry.addData("y", y);



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
        if(rot == 0 && !checkStop){
            double error = target - heading;
            if (error > 180)
                error -= 360;
            else if (error < -180)
                error += 360;
            if(lastTime > 0)
                derivative = (error - lastError) / (timer.milliseconds() - lastTime);
            integralSum += error;
            integralSum *= Math.signum(error);


            if(Math.abs(error) > 0.5) // prob make it some small decimal
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
    public void resetPID(){
        timer.reset();
        derivative = 0;
        integralSum = 0;
        lastTime = 0;
        lastError = 0;
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
        //telemetry.addData("gyro", orientation.firstAngle);
        // return orientation.firstAngle;
        return currAngle;
    }

    public double getAbsoluteAngle()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }




}
