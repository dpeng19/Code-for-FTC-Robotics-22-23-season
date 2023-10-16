package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "RedRightAuto")
public class RedRightAuto extends LinearOpMode {


    //lift positions-change these after determing right tick amount
    private final double INITIAL_RAISE = 300;
    private final double LOW_TARGET = 1850;
    private final double MED_TARGET = 3100;
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
    private double ConeHeight = 660;
    private final double coneDecrease = 1.20 * TICKS_PER_INCH;
    private boolean isLifted = false;

    //ElapsedTime totalTimer = new ElapsedTime();

    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime driveTimer = new ElapsedTime();
    /*
    private double derivative = 0;
    private double integralSum = 0;
    private double lastTime = 0;
    private double lastError = 0;
    public static double p = 0.015, i = 0, d = 0.05;
    private double targetHead = 0;

     */
    boolean isFirstDecrease = true;


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
        WAIT_4,
        TRAJ_6,
        WAIT_5,
        TRAJ_7,
        WAIT_6,
        TRAJ_8,
        WAIT_7,
        TRAJ_9,
        WAIT_8,
        TRAJ_10,
        WAIT_9,
        TRAJ_11,
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
    Pose2d startPose = new Pose2d(33, -63.5, Math.toRadians(0));
    //Pose2d startPose = new Pose2d(35, -63.5, Math.toRadians(0));//try 32

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Scanner scanner = new Scanner(hardwareMap, telemetry, false);
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
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 20;
                    }
                })
                .splineToConstantHeading(new Vector2d(13.1, -55.5), Math.toRadians(180))//set up closer to center
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30;
                    }
                })

                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .setReversed(false)
                .strafeLeft(31.8)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeLeft(6.2)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 20;
                    }
                })
                .splineToConstantHeading(new Vector2d(20, -12.7), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30;
                    }
                })
                .forward(39.2)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(48.9, -12.3, Math.toRadians(-90.5)))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(59, -12.3, Math.toRadians(0)))//change back to x = -59.5 if inconsistent
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(49.1, -12.3, Math.toRadians(-90.5)))//change back to y = -12.7 if inconsistent
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(59, -12.1, Math.toRadians(0)))//change back to x = -59.5 if inconsistent
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(49.2, -12.6, Math.toRadians(-90.5)))//change back to y = -12.7 if inconsistent
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(59, -12.1, Math.toRadians(0)))//change back to x = -59.5 if inconsistent
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineToLinearHeading(new Pose2d(49.1, -12.75, Math.toRadians(-90.5)))//change back to y = -12.7 if inconsistent
                .build();
        TrajectorySequence loc1 = drive.trajectorySequenceBuilder(traj10.end())
                .lineToConstantHeading(new Vector2d(14, -11))
                //.turn(Math.toRadians(-183))
                .forward(4)
                .build();
        TrajectorySequence loc2 = drive.trajectorySequenceBuilder(traj10.end())
                .strafeRight(11)
                //.turn(Math.toRadians(-180.5))
                .forward(3)
                .build();
        TrajectorySequence loc3 = drive.trajectorySequenceBuilder(traj10.end())
                //.strafeLeft(4)
                //.splineToConstantHeading(new Vector2d(59, -34.5), Math.toRadians(-90))
                //.turn(Math.toRadians(-181))
                .strafeLeft(9)
                .forward(4)
                .build();
        /*
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(-95)))
                .build();

         */


        //initializeIMU();
        /*
        claw.close();
        if(initTimer.seconds() > delay){
            lift.setTarget(INITIAL_RAISE);
        }

         */
        waitForStart();
        claw.close(); //maybe put before waitForStart()
        Color color = scanner.getResult();
        scanner.stop();

        if (isStopRequested()) return;
        //lift.setTarget(0);
        //trajstate = TrajectoryState.Traj1;
        //drive.followTrajectorySequenceAsync(traj1);
        // trajstate = TrajectoryState.TRAJ_1;
        // drive.followTrajectorySequenceAsync(traj1);
        initTimer.reset();
        //totalTimer.reset();
        //timer.reset();
        while (opModeIsActive() && !isStopRequested()) {

            switch (trajstate) {

                case START:
                    if (initTimer.seconds() > 0.7) {
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
                    if(driveTimer.seconds() > 0.4 && isFirstDecrease){
                        lift.setTarget(ConeHeight);
                        ConeHeight -= coneDecrease;
                        liftState = LiftState.RETRACT;
                        isFirstDecrease = false;
                    }
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_2;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_2:
                    if(initTimer.seconds() > delay && !isLifted){
                        liftTimer.reset();
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                        isLifted = true;
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
                        trajstate = TrajectoryState.WAIT_4;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_4:
                    if(initTimer.seconds() > delay && !isLifted){
                        liftTimer.reset();
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                        isLifted = true;
                    }
                    break;
                case TRAJ_6:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_5;
                    }
                    break;
                case WAIT_5:
                    break;
                case TRAJ_7:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_6;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_6:
                    if(initTimer.seconds() > delay && !isLifted){
                        liftTimer.reset();
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                        isLifted = true;
                    }
                    break;
                case TRAJ_8:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_7;
                    }
                    break;
                case WAIT_7:
                    break;
                case TRAJ_9:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_8;
                        claw.close();
                        initTimer.reset();
                    }
                    break;
                case WAIT_8:
                    if(initTimer.seconds() > delay && !isLifted){
                        liftTimer.reset();
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                        isLifted = true;
                    }
                    break;
                case TRAJ_10:
                    if(!drive.isBusy()){
                        trajstate = TrajectoryState.WAIT_9;
                    }
                    break;
                case WAIT_9:
                    break;
                case TRAJ_11:
                    if(!drive.isBusy()){
                        //drivestate = DriveState.MANUAL;
                        //targetHead = 90;
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

            if(firstRun && initTimer.seconds() > 0.5){
                lift.setTarget(INITIAL_RAISE);
                firstRun = false;
            }


            switch (liftState) {
                case START:
                    //if(trajstate == TrajectoryState.Traj2){
                    //if(Math.abs(claw.claw1.getPosition() - 0.87) < 0.01 && Math.abs(claw.claw2.getPosition()) < 0.01){
                    if (trajstate == TrajectoryState.TRAJ_1 && traj1.duration() - driveTimer.seconds() < 0.3) {
                        lift.setTarget(MED_TARGET);
                        liftState = LiftState.EXTEND;
                    }
                    /*
                    else if(trajstate == TrajectoryState.TRAJ_4 || trajstate == TrajectoryState.TRAJ_6 ||
                            trajstate == TrajectoryState.TRAJ_8 || trajstate == TrajectoryState.TRAJ_10){
                        lift.setTarget(LOW_TARGET);
                        liftState = LiftState.EXTEND;
                    }

                     */

                    break;
                case EXTEND:
                    if(liftTimer.seconds() > 0.3 && trajstate == TrajectoryState.WAIT_2){
                        trajstate = TrajectoryState.TRAJ_4;
                        drive.followTrajectorySequenceAsync(traj4);
                    }
                    if(liftTimer.seconds() > 0.3 && trajstate == TrajectoryState.WAIT_4){
                        trajstate = TrajectoryState.TRAJ_6;
                        drive.followTrajectorySequenceAsync(traj6);
                    }
                    if(liftTimer.seconds() > 0.3 && trajstate == TrajectoryState.WAIT_6){
                        trajstate = TrajectoryState.TRAJ_8;
                        drive.followTrajectorySequenceAsync(traj8);
                    }
                    if(liftTimer.seconds() > 0.3 && trajstate == TrajectoryState.WAIT_8){
                        trajstate = TrajectoryState.TRAJ_10;
                        drive.followTrajectorySequenceAsync(traj10);
                    }
                    if ((Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) && (trajstate == TrajectoryState.WAIT_1 ||
                            trajstate == TrajectoryState.WAIT_3 ||  trajstate == TrajectoryState.WAIT_5 ||  trajstate == TrajectoryState.WAIT_7 || trajstate == TrajectoryState.WAIT_9
                    )) {
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
                            driveTimer.reset();
                        }
                        else if (trajstate == TrajectoryState.WAIT_3) {
                            drive.followTrajectorySequenceAsync(traj5);
                            trajstate = TrajectoryState.TRAJ_5;
                        }
                        else if (trajstate == TrajectoryState.WAIT_5) {
                            drive.followTrajectorySequenceAsync(traj7);
                            trajstate = TrajectoryState.TRAJ_7;
                        }
                        else if (trajstate == TrajectoryState.WAIT_7) {
                            drive.followTrajectorySequenceAsync(traj9);
                            trajstate = TrajectoryState.TRAJ_9;
                        }
                        else if (trajstate == TrajectoryState.WAIT_9) {
                            if(color == Color.GREEN)
                                drive.followTrajectorySequenceAsync(loc1);
                            else if(color == Color.RED)
                                drive.followTrajectorySequenceAsync(loc2);
                            else if(color == Color.YELLOW)
                                drive.followTrajectorySequenceAsync(loc3);
                            else
                                drive.followTrajectorySequenceAsync(loc2);
                            trajstate = TrajectoryState.TRAJ_11;
                          //  lift.setTarget(-40);
                            lift.setTarget(-10);
                            liftState = LiftState.RETRACT;
                        }
                        if(trajstate != TrajectoryState.TRAJ_11 && trajstate != TrajectoryState.TRAJ_3) {
                            lift.setTarget(ConeHeight);
                            ConeHeight -= coneDecrease;
                            liftState = LiftState.RETRACT;
                        }

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
                        isLifted = false;
                    }
                    break;
            }
            lastPos = lift.slide.getCurrentPosition();

            //if(drivestate == DriveState.AUTO)
            drive.update();
            /*
            if(drivestate == DriveState.MANUAL){
                if(totalTimer.seconds() < 29.2) {
                    Pose2d power = getPower(drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, targetHead);
                    drive.setWeightedDrivePower(power);
                }
                else
                    drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            }

             */

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
            telemetry.update();


        }
    }
    /*
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

