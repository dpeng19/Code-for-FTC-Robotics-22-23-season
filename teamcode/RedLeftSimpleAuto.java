package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Vision.Color;
import org.firstinspires.ftc.teamcode.Vision.Scanner;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
@Disabled
@Autonomous(name = "RedLeftSimpleAuto")
public class RedLeftSimpleAuto extends LinearOpMode {


    //lift positions-change these after determing right tick amount
    private final double INITIAL_RAISE = 300;
    private final double LOW_TARGET = 1850;
    private final double MED_TARGET = 2900;
    private final double HIGH_TARGET = 4180;

    private final double INCHES_PER_REV = 4.40945;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    private double lastPos = 0;

    private final double delay = 0.5;
    ElapsedTime initTimer = new ElapsedTime();
    private final double STACK_TIME = 0.3;
    ElapsedTime StackTimer = new ElapsedTime();

    private boolean firstRun = true;
    private double ConeHeight = 635;
    private final double coneDecrease = 1.25 * TICKS_PER_INCH;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime driveTimer = new ElapsedTime();
    ElapsedTime totalTimer = new ElapsedTime();
    private double derivative = 0;
    private double integralSum = 0;
    private double lastTime = 0;
    private double lastError = 0;
    public static double p = 0.015, i = 0, d = 0.05;
    private double targetHead = 0;


    enum TrajectoryState {
        START,
        TRAJ_1,
        TRAJ_2,
        WAIT_1,
        TRAJ_3,
        WAIT_2,
        TRAJ_4,
        WAIT_3,
        TRAJ_5,
        // WAIT_8,
        // TRAJ_10,
        //WAIT_9,
        IDLE
    }

    enum LiftState {
        START,
        EXTEND,
        STACK,
        RETRACT
    }

    enum DriveState{
        AUTO,
        MANUAL
    }



    TrajectoryState trajstate = TrajectoryState.START;
    LiftState liftState = LiftState.START;
    DriveState drivestate = DriveState.AUTO;
    Pose2d startPose = new Pose2d(-32, -63.5, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Scanner scanner = new Scanner(hardwareMap, telemetry, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
/*
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

 */
/*
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

 */



        //slide.setDirection(DcMotorEx.Direction.REVERSE);


        drive.setPoseEstimate(startPose);







        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.8, -55.5), Math.toRadians(0))//set up closer to center

                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setReversed(false)
                .strafeRight(32)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeRight(6)
                .splineToConstantHeading(new Vector2d(-20, -13), Math.toRadians(180))
                .forward(39)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-48.6, -12.5, Math.toRadians(-91.5)))
                .build();
        TrajectorySequence loc1 = drive.trajectorySequenceBuilder(traj4.end())
                .strafeRight(3)
                .splineToConstantHeading(new Vector2d(-59, -37.5), Math.toRadians(-90))
                .turn(Math.toRadians(181))
                .build();
        TrajectorySequence loc2 = drive.trajectorySequenceBuilder(traj4.end())
                .strafeLeft(11)
                .turn(Math.toRadians(180.5))
                .build();
        TrajectorySequence loc3 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToConstantHeading(new Vector2d(-14, -12))
                .turn(Math.toRadians(183))
                .build();
        /*
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(-95)))
                .build();

         */


        //initializeIMU();
        claw.close();
        if(initTimer.seconds() > delay){
            lift.setTarget(INITIAL_RAISE);
        }
        waitForStart();
        //claw.close(); //maybe put before waitForStart()
        Color color = scanner.getResult();
        scanner.stop();

        if (isStopRequested()) return;
        //lift.setTarget(0);
        //trajstate = TrajectoryState.Traj1;
        //drive.followTrajectorySequenceAsync(traj1);
        // trajstate = TrajectoryState.TRAJ_1;
        // drive.followTrajectorySequenceAsync(traj1);
        initTimer.reset();
        totalTimer.reset();
        //timer.reset();
        while (opModeIsActive() && !isStopRequested()) {

            switch (trajstate) {

                case START:
                    if (initTimer.seconds() > 0.2) {
                        trajstate = TrajectoryState.TRAJ_1;
                        drive.followTrajectorySequenceAsync(traj1);
                        driveTimer.reset();
                    }
                    break;


                case TRAJ_1:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.TRAJ_2;
                        drive.followTrajectorySequenceAsync(traj2);
                    }
                    break;
                case TRAJ_2:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.WAIT_1;
                    }
                    break;
                case WAIT_1:
                    break;
                case TRAJ_3:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_2;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_2:
                    if(initTimer.seconds() > delay){
                        trajstate = TrajectoryState.TRAJ_4;
                        drive.followTrajectorySequenceAsync(traj4);
                    }
                    break;
                case TRAJ_4:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_3;
                    }
                    break;
                case WAIT_3:
                    break;
                case TRAJ_5:
                    if(!drive.isBusy()){
                        drivestate = DriveState.MANUAL;
                        targetHead = -90;
                        trajstate = TrajectoryState.IDLE;
                    }
                    break;

                    /*
                case TRAJ_9:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_8;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_8:
                    if(initTimer.seconds() > delay){
                        trajstate = TrajectoryState.TRAJ_10;
                        drive.followTrajectorySequenceAsync(traj10);
                    }
                    break;
                case TRAJ_10:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_9;
                    }
                    break;
                case WAIT_9:
                    break;

                     */
                case IDLE:
                    break;


                     /*
                    case Wait5:
                        break;
                    case Traj7:
                        if(!drive.isBusy()){
                            claw.close();
                            trajstate = TrajectoryState.Wait6;
                            initTimer.reset();
                            firstRun = true;
                        }
                        break;

                      */

                                /*
                    case Wait6:
                        if(initTimer.seconds() > delay){
                            drive.followTrajectorySequenceAsync(traj8);
                            trajstate = TrajectoryState.Traj8;
                        }
                        break;
                    case Traj8:
                        if(!drive.isBusy()){
                            trajstate = TrajectoryState.IDLE;
                        }

                                 */
            }



            telemetry.addData("initTimer", initTimer.seconds());
            telemetry.addData("claw1pos", claw.claw1.getPosition());
            telemetry.addData("claw2pos", claw.claw2.getPosition());
            telemetry.addData("slidepos", lift.slide.getCurrentPosition());
            /*
            if(firstRun && initTimer.seconds() > delay){
                lift.setTarget(INITIAL_RAISE);
                firstRun = false;
            }

             */
            switch (liftState) {
                case START:
                    //if(trajstate == TrajectoryState.Traj2){
                    //if(Math.abs(claw.claw1.getPosition() - 0.87) < 0.01 && Math.abs(claw.claw2.getPosition()) < 0.01){
                    if (trajstate == TrajectoryState.TRAJ_1 && traj1.duration() - driveTimer.seconds() < 0.3) {
                        lift.setTarget(MED_TARGET);
                        liftState = LiftState.EXTEND;
                    }
                    else if(trajstate == TrajectoryState.TRAJ_4){
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                    }

                    break;
                case EXTEND:
                    if ((Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) && (trajstate == TrajectoryState.WAIT_1 ||
                            trajstate == TrajectoryState.WAIT_3)) {
                        claw.open();
                        StackTimer.reset();
                        liftState = LiftState.STACK;
                    }
                    break;
                case STACK:
                    if (StackTimer.seconds() >= STACK_TIME) { //may have to reduce STACK_TIME for optimization
                        if (trajstate == TrajectoryState.WAIT_1) {
                            drive.followTrajectorySequenceAsync(traj3);
                            trajstate = TrajectoryState.TRAJ_3;
                        }
                        else if (trajstate == TrajectoryState.WAIT_3) {
                            if(color == Color.GREEN)
                                drive.followTrajectorySequenceAsync(loc1);
                            else if(color == Color.RED)
                                drive.followTrajectorySequenceAsync(loc2);
                            else if(color == Color.YELLOW)
                                drive.followTrajectorySequenceAsync(loc3);
                            else
                                drive.followTrajectorySequenceAsync(loc2);
                            trajstate = TrajectoryState.TRAJ_5;
                            lift.setTarget(-40);
                        }

                        if(trajstate != TrajectoryState.TRAJ_5) {
                            lift.setTarget(ConeHeight);
                            ConeHeight -= coneDecrease;
                        }
                        liftState = LiftState.RETRACT;
                        /*
                            /*
                            if (trajstate != TrajectoryState.Traj4 && trajstate != TrajectoryState.IDLE) {
                                drive.followTrajectorySequenceAsync(traj3);
                                trajstate = TrajectoryState.Traj3;
                            }

                        if (trajstate == TrajectoryState.Wait1) {
                            drive.followTrajectorySequenceAsync(traj3);
                            trajstate = TrajectoryState.Traj3;
                        } else if (trajstate == TrajectoryState.Wait3) {
                            drive.followTrajectorySequenceAsync(traj5);
                            trajstate = TrajectoryState.Traj5;
                        } else if (trajstate == TrajectoryState.Wait5) {
                            drive.followTrajectorySequenceAsync(traj7);
                            trajstate = TrajectoryState.Traj7;
                        }

                            else if (trajstate == TrajectoryState.Wait5) {
                                drive.followTrajectorySequenceAsync(traj7);
                                trajstate = TrajectoryState.Traj7;
                            }

                             */

                    }
                    break;
                case RETRACT:
                    if (Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) {
                        liftState = LiftState.START;
                    }
                    break;
            }
            lastPos = lift.slide.getCurrentPosition();

            //if(drivestate == DriveState.AUTO)
            drive.update();
            if(drivestate == DriveState.MANUAL){
                if(totalTimer.seconds() < 29.2) {
                    Pose2d power = getPower(drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, targetHead);
                    drive.setWeightedDrivePower(power);
                }
                else
                    drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            }

            // else
            //setPower(getAbsoluteAngle(), targetHead);

            lift.update();


            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("trajstate", trajstate);
            telemetry.addData("lifttarget", lift.getTarget());
            telemetry.addData("liftpos", lift.slide.getCurrentPosition());
            telemetry.addData("Color", color);
            telemetry.addData("imu", drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("targetHead", targetHead);
            telemetry.update();


        }
    }
    public Pose2d getPower(double heading, double target) {


        //keeps target between -180 to 180
        if (target > 180)
            target -= 360;
        else if (target  < -180)
            target += 360;




        double correction = 0;
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



        lastError = error;
        lastTime = timer.milliseconds();
        return new Pose2d(0, 0, correction * 0.6);







    }
    /*
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
    public double getAbsoluteAngle()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



     */

}
