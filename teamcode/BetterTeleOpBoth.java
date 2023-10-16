package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

//@Config
@Disabled
@TeleOp(name = "BetterTeleOpBoth", group = "TeleOp")
public class BetterTeleOpBoth extends LinearOpMode {

    //this mode limits robot movement to forwards, backwards, straferight, strafeleft
    //helps with gamepad sensitivity
    boolean isSimpleMode = true;

    //stores powers for the motors (y = -1 -> forward, y = 1 -> backward, x = 1 -> right, x = -1 -> left)
    private int[][] dir = {{0, 1},{-1, 0},{0, -1}, {1, 0}};

    //private DcMotorEx FR, FL, BR, BL;
    public static double pwr_Mult = 0.6; //speed of robot, ranges from -1 to 1, adjust closer to 1 for faster speed

    //initialize IMU variables
    //private BNO055IMU imu;
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
    private boolean pGA2B = false;
    private boolean pGA2X = false;
    private boolean pGA2Y = false;
    private boolean pGA2BU = false;
    private boolean dumpReady = false;
    private boolean pGA2A = false;
    private boolean isScored = false;
    private boolean pGA1LB = false;
    private boolean pGA2LY = false;
    private boolean pGA2LB = false;
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
    //private double correction = 0;

    private double lastLiftPos = 0;
    //may have to adjust
    private final double LOW_TARGET = 1800;
    private final double MED_TARGET = 2950;
    private final double HIGH_TARGET = 4200;

    private double angleOffset = 0;
    private boolean needReset = false;

    ElapsedTime liftTimer = new ElapsedTime();
    // ElapsedTime driveTimer = new ElapsedTime();

    boolean disableAuto = false;
    boolean isFirstLift = true;
    enum DriveState{
        MANUAL,
        AUTO
    }
    /*
        enum LiftState {
            START,
            EXTEND,
            STACK,
            RETRACT
        }
        LiftState liftState = LiftState.START;

     */
    enum ScoreState{
        LOW,
        MED,
        HIGH;
    }
    enum Side{
        RIGHT,
        LEFT
    }
    DriveState driveState = DriveState.MANUAL;
    ScoreState scoreState;
    Side side = Side.LEFT;

    private final Pose2d leftCollectPose = new Pose2d(-12, -57, Math.toRadians(-90));
    private final Pose2d leftLowToLowMedBackStart = new Pose2d(-24, -35.5, Math.toRadians(-90));
    private final Pose2d leftMedToMedLowBackStart = new Pose2d(-11, -25, Math.toRadians(-180));
    private final Pose2d leftHighToLowMedBackStart = new Pose2d(-12.5, -24, Math.toRadians(0));
    private final Pose2d leftHighScoreStartAlt = new Pose2d(-10, -58, Math.toRadians(0));
    private final Pose2d leftHighToHighBackStart= new Pose2d(-12, -23.5, Math.toRadians(0));

    private final Pose2d rightCollectPose = new Pose2d(12, -57, Math.toRadians(-90));
    private final Pose2d rightLowToLowMedBackStart = new Pose2d(24, -35.5, Math.toRadians(-90));
    private final Pose2d rightMedToMedLowBackStart = new Pose2d(11, -25, Math.toRadians(0));
    private final Pose2d rightHighToLowMedBackStart = new Pose2d(12.5, -24, Math.toRadians(180));
    private final Pose2d rightHighScoreStartAlt = new Pose2d(10, -58, Math.toRadians(180));
    private final Pose2d rightHighToHighBackStart= new Pose2d(12, -23.5, Math.toRadians(180));

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        /*
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

         */
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);// may want to adjust timeout admission error to less time alloted
        //RUN_USING_ENCODER mode helps motors maintain equal speeds to each other
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);


        //LOW-THERE
        TrajectorySequence leftLowScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .setReversed(true)
                .back(12)
                .splineToConstantHeading(new Vector2d(-26.5, -39.5), Math.toRadians(-180))
                .build();
        //LOW-BACK
        TrajectorySequence leftLowToLowMedBack =  drive.trajectorySequenceBuilder(new Pose2d(-24, -35.5, Math.toRadians(-90)))
                .strafeLeft(4)
                .splineToConstantHeading(new Vector2d(-12, -57), Math.toRadians(-90))
                .build();
        //MED-THERE
        TrajectorySequence leftMedScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .back(12)
                .lineToLinearHeading(new Pose2d(-11.5, -25, Math.toRadians(-180)))
                .build();

        //MED-BACK
        TrajectorySequence leftMedToMedLowBack = drive.trajectorySequenceBuilder(new Pose2d(-11, -25, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-13, -34,  Math.toRadians(-94)))
                .forward(22.5)
                .build();

        //HIGH-THERE
        TrajectorySequence leftHighScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .back(18)
                .lineToLinearHeading(new Pose2d(-13, -26, Math.toRadians(0)))
                .build();


        //HIGH-BACK
        TrajectorySequence leftHighToLowMedBack = drive.trajectorySequenceBuilder(new Pose2d(-12.5, -24, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-10, -34,  Math.toRadians(-94)))
                .forward(21)
                .build();

        //HIGH-THERE-ALT
        TrajectorySequence leftHighScoreAlt = drive.trajectorySequenceBuilder(new Pose2d(-10, -58, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12, -56), Math.toRadians(90))
                .strafeLeft(32.5)
                .build();


        //HIGH-BACK-ALT
        TrajectorySequence leftHighToHighBack = drive.trajectorySequenceBuilder(new Pose2d(-12, -23.5, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-13.5, -28), Math.toRadians(-90))
                .strafeRight(30)
                .build();


        //Right
        //LOW-THERE
        TrajectorySequence rightLowScore = drive.trajectorySequenceBuilder(rightCollectPose)
                .setReversed(true)
                .back(12)
                .splineToConstantHeading(new Vector2d(26.5, -39.5), Math.toRadians(0))
                .build();
        //LOW-BACK
        TrajectorySequence rightLowToLowMedBack =  drive.trajectorySequenceBuilder(rightLowToLowMedBackStart)
                .strafeRight(4)
                .splineToConstantHeading(new Vector2d(12, -57), Math.toRadians(-90))
                .build();
        //MED-THERE
        TrajectorySequence rightMedScore = drive.trajectorySequenceBuilder(rightCollectPose)
                .back(12)
                .lineToLinearHeading(new Pose2d(11.5, -25, Math.toRadians(0)))
                .build();

        //MED-BACK
        TrajectorySequence rightMedToMedLowBack = drive.trajectorySequenceBuilder(rightMedToMedLowBackStart)
                .lineToLinearHeading(new Pose2d(13, -34,  Math.toRadians(-86)))
                .forward(22.5)
                .build();

        //HIGH-THERE
        TrajectorySequence rightHighScore = drive.trajectorySequenceBuilder(rightCollectPose)
                .back(18)
                .lineToLinearHeading(new Pose2d(13, -26, Math.toRadians(-180)))
                .build();


        //HIGH-BACK
        TrajectorySequence rightHighToLowMedBack = drive.trajectorySequenceBuilder(rightHighToLowMedBackStart)
                .lineToLinearHeading(new Pose2d(10, -34,  Math.toRadians(-90)))
                .forward(21)
                .build();

        //HIGH-THERE-ALT
        TrajectorySequence rightHighScoreAlt = drive.trajectorySequenceBuilder(rightHighScoreStartAlt)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(12, -56), Math.toRadians(90))
                .strafeRight(32.5)
                .build();


        //HIGH-BACK-ALT
        TrajectorySequence rightHighToHighBack = drive.trajectorySequenceBuilder(rightHighToHighBackStart)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13.5, -28), Math.toRadians(-90))
                .strafeLeft(30)
                .build();






        //initializeIMU();

        timer.reset();
        waitForStart();

        //FTC dashboard for some cool graphs and PID tuning
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;
        //claw.close();
        //lift.setTarget(0);
        //resetAngle();
        targetHead = 0;
        //imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle = 90;
        while (opModeIsActive()) {
            drive.update();
            boolean ga2A = gamepad2.a;
            switch (driveState){
                case MANUAL:
                    Pose2d power = getPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, pwr_Mult, isSimpleMode, drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angleOffset, targetHead);
                    drive.setWeightedDrivePower(power);
                    boolean ga1X = gamepad1.x;
                    if(ga1X && !pGA1X){
                        if(Math.abs(targetHead) != 180)
                            targetHead += 90;
                        else
                            targetHead = -90;
                        resetPID();
                        //isYPressed = true;


                    }
                    pGA1X = ga1X;
                    boolean ga1Y = gamepad1.y;
                    if(ga1Y && !pGA1Y){
                        if(Math.abs(targetHead) != 180)
                            targetHead -= 90;
                        else
                            targetHead = 90;
                        resetPID();
                        //isXPressed = true;
                    }
                    pGA1Y = ga1Y;

                    boolean ga1A = gamepad1.a;
                    if(ga1A && !pGA1A){
                        if(pwr_Mult == 0.4 || pwr_Mult == 0.6)
                            pwr_Mult = 0.2;
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

                    boolean ga1RX = (gamepad1.right_stick_x != 0);
                    if(pGA1RX && !ga1RX ){
                        checkStop = true;
                    }
                    pGA1RX = ga1RX;

                    if(checkStop){ //make sure ckeckStop becomes false when stop
                        List<Double> wheelVel = drive.getWheelVelocities();
                        if(wheelVel.get(0) <= 0.01 && wheelVel.get(1) <= 0.01 && wheelVel.get(2) <= 0.01 && wheelVel.get(3) <= 0.01){
                            if(needReset){
                                angleOffset = -drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                                targetHead = 0;
                                needReset = false;
                            }
                            else
                                targetHead = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angleOffset;
                            resetPID();
                            checkStop = false;
                        }
                    }
                    boolean ga1LB = gamepad1.left_bumper;
                    if(ga1LB && !pGA1LB){
                        if(!checkStop) {
                            angleOffset = -drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                            targetHead = 0;
                            resetPID();
                        }
                        else
                            needReset = true;

                    }
                    pGA1LB = ga1LB;
                    if(!pGA2A && ga2A && dumpReady){
                        claw.open();
                        clawOpen = true;
                        lift.setTarget(-60);//may have to adjust
                        dumpReady = false;
                        isFirstLift = true;
                        //pwr_Mult = 0.6;
                    }
                    if(gamepad2.x){
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        if(!clawOpen){
                            if(!disableAuto && isFirstLift){
                                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftCollectPose);
                                    drive.followTrajectorySequenceAsync(leftLowScore);
                                }
                                else{
                                    drive.setPoseEstimate(rightCollectPose);
                                    drive.followTrajectorySequenceAsync(rightLowScore);
                                }
                                driveState = DriveState.AUTO;
                                isFirstLift = false;
                                targetHead = 180;
                                scoreState = ScoreState.LOW;
                            }
                            lift.setTarget(LOW_TARGET);
                        }
                        else if(clawOpen &&!disableAuto){
                            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            if(scoreState == ScoreState.LOW){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(leftLowToLowMedBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(rightLowToLowMedBack);
                                }
                            }
                            else if(scoreState == ScoreState.MED){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(leftMedToMedLowBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(rightMedToMedLowBack);

                                }
                            }
                            else if(scoreState == ScoreState.HIGH){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftHighToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(leftHighToLowMedBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightHighToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(rightHighToLowMedBack);
                                }
                            }

                            driveState = DriveState.AUTO;
                            targetHead = 180;
                        }
                    }
                    else if(gamepad2.y){
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        if(!clawOpen){
                            if(!disableAuto && isFirstLift){
                                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftCollectPose);
                                    drive.followTrajectorySequenceAsync(leftMedScore);
                                    targetHead = 90;
                                }
                                else{
                                    drive.setPoseEstimate(rightCollectPose);
                                    drive.followTrajectorySequenceAsync(rightMedScore);
                                    targetHead = -90;
                                }
                                driveState = DriveState.AUTO;
                                isFirstLift = false;
                                scoreState = ScoreState.MED;
                            }
                            lift.setTarget(MED_TARGET);
                        }
                        else if(clawOpen &&!disableAuto){
                            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            if(scoreState == ScoreState.LOW){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(leftLowToLowMedBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(rightLowToLowMedBack);
                                }
                            }
                            else if(scoreState == ScoreState.MED){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(leftMedToMedLowBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(rightMedToMedLowBack);

                                }
                            }
                            else if(scoreState == ScoreState.HIGH){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftHighToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(leftHighToLowMedBack);
                                }
                                else{
                                    drive.setPoseEstimate(rightHighToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(rightHighToLowMedBack);
                                }
                            }

                            driveState = DriveState.AUTO;
                            targetHead = 180;
                        }
                    }
                    else if(gamepad2.b){
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                        if(!clawOpen){
                            if(!disableAuto && isFirstLift){
                                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                //change to other high path when applicable
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftHighScoreStartAlt);
                                    drive.followTrajectorySequenceAsync(leftHighScoreAlt);
                                    targetHead = -90;
                                }
                                else{
                                    drive.setPoseEstimate(rightHighScoreStartAlt);
                                    drive.followTrajectorySequenceAsync(rightHighScoreAlt);
                                    targetHead = 90;
                                }
                                driveState = DriveState.AUTO;
                                isFirstLift = false;
                                scoreState = ScoreState.HIGH;
                            }
                            lift.setTarget(HIGH_TARGET);
                        }
                        else if(clawOpen &&!disableAuto){
                            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            if(scoreState == ScoreState.HIGH){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftHighToHighBackStart);
                                    drive.followTrajectorySequenceAsync(leftHighToHighBack);
                                    targetHead = -90;

                                }
                                else{
                                    drive.setPoseEstimate(rightHighToHighBackStart);
                                    drive.followTrajectorySequenceAsync(rightHighToHighBack);
                                    targetHead = 90;
                                }

                            }
                            if(scoreState == ScoreState.LOW){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(leftLowToLowMedBack);
                                    targetHead = -90;
                                }
                                else{
                                    drive.setPoseEstimate(rightLowToLowMedBackStart);
                                    drive.followTrajectorySequenceAsync(rightLowToLowMedBack);
                                    targetHead = 90;
                                }
                            }
                            else if(scoreState == ScoreState.MED){
                                if(side == Side.LEFT) {
                                    drive.setPoseEstimate(leftMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(leftMedToMedLowBack);
                                    targetHead = -90;
                                }
                                else{
                                    drive.setPoseEstimate(rightMedToMedLowBackStart);
                                    drive.followTrajectorySequenceAsync(rightMedToMedLowBack);
                                    targetHead = 90;

                                }
                            }
                            driveState = DriveState.AUTO;

                        }
                    }
                    break;
                case AUTO:
                    if(drive.isBusy() && !pGA2A && ga2A){
                        drive.breakFollowing();
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        driveState = DriveState.MANUAL;
                        pwr_Mult = 0.6;
                    }
                    else if(!drive.isBusy()){
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        driveState = DriveState.MANUAL;
                        if(clawOpen)
                            pwr_Mult  = 0.4;
                        else
                            pwr_Mult = 0.2;
                    }
                    break;


            }
            pGA2A = ga2A;







            if((Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition()  - lastLiftPos) <= 1) && lift.getTarget() > 500 && Math.abs(-gamepad2.left_stick_y) < 0.00001) {
                //pwr_Mult = 0.3;//maybe change this to faster
                dumpReady = true;

            }
            /*
            boolean ga2A = gamepad2.a;
            if(ga2A && !pGA2A && dumpReady){
                claw.open();
                clawOpen = true;
                lift.setTarget(-30);//may have to adjust
                dumpReady = false;
                isFirstLift = true;
                pwr_Mult = 0.6;
            }
            pGA2A = ga2A;

             */

            lastLiftPos = lift.slide.getCurrentPosition();
            //manual lift
            boolean ga2LY = (Math.abs(-gamepad2.left_stick_y) >= 0.00001);
            if(!ga2LY){
                if(pGA2LY)
                    lift.setTarget(lift.slide.getCurrentPosition());
                lift.update();
            }
            //test this
            else{
                if(-gamepad2.left_stick_y > 0)
                    lift.slide.setPower(-gamepad2.left_stick_y * 0.85);
                else
                    lift.slide.setPower(-gamepad2.left_stick_y * 0.6);
            }
            pGA2LY = ga2LY;

/*
            if(!drive.isBusy() && driveState == DriveState.AUTO && driveTimer.seconds() > 0.5)
            {
                driveState = DriveState.MANUAL;
                prepareManual();
            }

 */






            boolean ga2B = gamepad2.right_bumper;
            if(ga2B && !pGA2B){
                disableAuto = !disableAuto;
            }
            pGA2B = ga2B;

            boolean ga2LB = gamepad2.left_bumper;
            if(ga2LB && !pGA2LB){
                if(side == Side.LEFT)
                    side = Side.RIGHT;
                else
                    side = Side.LEFT;
            }
            pGA2LB = ga2LB;
            //telemetry.addData("liftState", liftState);
            telemetry.addData("x", gamepad2.x);
            telemetry.addData("y", gamepad2.y);
            telemetry.addData("b", gamepad2.b);


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
                    pwr_Mult = 0.2;
                }
                else{
                    lift.setTarget(175);
                    pwr_Mult = 0.6;
                }
                needLift = false;

            }
            telemetry.addData("liftPower", lift.slide.getPower());
            telemetry.addData("liftPosition", lift.slide.getCurrentPosition());
            telemetry.addData("liftTarget", lift.getTarget());
            telemetry.addData("Heading", normalizeHeading(drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angleOffset));
            telemetry.addData("Target", targetHead);
            telemetry.addData("HeadingState", Heading_State);
            telemetry.addData("Autodrive Disabled", disableAuto);//maybe make this on/off
            telemetry.addData("drivestate", driveState);
            telemetry.addData("clawOpen", clawOpen);
            telemetry.addData("isFirstLift", isFirstLift);
            telemetry.addData("isBusy", drive.isBusy());
            telemetry.addData("Side", side);
            telemetry.update();

        }
    }

    public Pose2d getPower(double x, double y, double rot, double pwr_Mult, boolean isSimpleMode, double heading, double target) {


        //keeps target between -180 to 180
        if (target > 180)
            target -= 360;
        else if (target  < -180)
            target += 360;

        if (heading > 180)
            heading -= 360;
        else if (heading  < -180)
            heading += 360;



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
        }
        telemetry.addData("xinit", x);
        telemetry.addData("yinit", y);
        if (isSimpleMode) {
            if (x != 0 || y != 0) {
                double angle = Math.atan2(y, x);
                angle = Math.toDegrees(angle);
                int angleI = (int) (Math.round(angle));
                telemetry.addData("angleI", angleI);
                double maxPower = Math.max(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y));
                //press up on left gamepad
                if (angleI > 45 && angleI <= 135) {
                    x = dir[Heading_State][0] * maxPower;
                    y = dir[Heading_State][1] * maxPower;

                }
                //press left on left gamepad
                else if ((angleI > 135 && angleI <= 180) || (angleI >= -180 && angleI <= -135)) {
                    x = dir[(Heading_State + 1) % 4][0] * maxPower;
                    y = dir[(Heading_State + 1) % 4][1] * maxPower;

                }
                //press down on left gamepad
                else if (angleI > -135 && angleI <= -45) {
                    x = dir[(Heading_State + 2) % 4][0] * maxPower;
                    y = dir[(Heading_State + 2) % 4][1] * maxPower;

                }
                //press right on left gamepad
                else if ((angleI >= 0 && angleI <= 45) || (angleI > -45 && angleI < 0)) {
                    //maybe change instead of 0 and 1s with x and ys
                    x = dir[(Heading_State + 3) % 4][0] * maxPower;
                    y = dir[(Heading_State + 3) % 4][1] * maxPower;

                }

            }
            telemetry.addData("x", x);
            telemetry.addData("y", y);



        }
        /*
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1); //maybe replace this with alternate
        double FRPower = (y - x - rot) / denominator * pwr_Mult;
        double FLPower = (y + x + rot) / denominator * pwr_Mult;
        double BRPower = (y + x - rot) / denominator * pwr_Mult;
        double BLPower = (y - x + rot) / denominator * pwr_Mult;

         */
        //alternatively
        /*
        double denominator = Math.max(Math.max(Math.abs(FRPower), Math.abs(BLPower)), Math.max(Math.abs(FLPower), Math.abs(BRPower)));
        */
        //double normalize = 1;
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
/*
            double dividePID = Math.max(Math.max(Math.abs(FRPower + correction), Math.abs(BLPower - correction)),
                    Math.max(Math.abs(FLPower - correction), Math.abs(BRPower + correction)));
            normalize = Math.max(dividePID, 1);

 */



            lastError = error;
            lastTime = timer.milliseconds();
            return new Pose2d(y * pwr_Mult, -x * pwr_Mult, correction);
        }
        return new Pose2d(y, -x, -rot * pwr_Mult);

        /*
        FR.setPower((FRPower + correction) / normalize);
        FL.setPower((FLPower - correction) / normalize);
        BR.setPower((BRPower + correction) / normalize);
        BL.setPower((BLPower - correction) / normalize);

         */
        // drive.setWeightedDrivePower()




    }

    public void resetPID(){
        timer.reset();
        derivative = 0;
        integralSum = 0;
        lastTime = 0;
        lastError = 0;
    }
    public double normalizeHeading(double heading){
        if (heading > 180)
            heading -= 360;
        else if (heading  < -180)
            heading += 360;
        return heading;
    }






}
