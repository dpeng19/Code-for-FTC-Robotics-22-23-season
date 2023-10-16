package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Disabled
@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {


    //lift positions-change these after determing right tick amount
    private final double LOW_TARGET = 1800;
    private final double MED_TARGET = 3000;
    private final double HIGH_TARGET = 4180;

    private final double INCHES_PER_REV = 4.40945;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    private double lastPos = 0;

    private final double delay = 0.5;
    ElapsedTime initTimer = new ElapsedTime();
    private final double STACK_TIME = 0.5;
    ElapsedTime StackTimer = new ElapsedTime();

    private boolean firstRun = true;
    private double ConeHeight = 600;
    private final double coneDecrease = 1.25 * TICKS_PER_INCH;


    enum TrajectoryState {
        Start,
        Traj1,
        Traj2,
        Wait1,
        Traj3,
        Wait2,
        Traj4,
        Wait3,
        Traj5,
        Wait4,
        Traj6,
        Wait5,
        Traj7,
        /*
        Traj7,
        Wait6,
        Traj8,

         */
        IDLE
    }

    enum LiftState {
        START,
        EXTEND,
        STACK,
        RETRACT
    }

    TrajectoryState trajstate = TrajectoryState.Start;
    LiftState liftState = LiftState.START;
    Pose2d startPose = new Pose2d(-39, -63.5, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
/*
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            */


        //slide.setDirection(DcMotorEx.Direction.REVERSE);


        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-11.7, -61), Math.toRadians(0))
                //.forward(24)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .strafeLeft(39.5)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .strafeLeft(4)
                .splineTo(new Vector2d(-34, -12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-24.2, -9.9, Math.toRadians(91.5)))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-57, -9.5, Math.toRadians(178)))
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-25, -8.3, Math.toRadians(91.5)))
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .strafeLeft(33)
                .back(25)
                .build();
            /*
            TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(-56.3, -9.0, Math.toRadians(176)))
                .build();
            TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(-25, -9.7, Math.toRadians(94)))
                .build();

             */

        waitForStart();
        claw.close();
        if (isStopRequested()) return;
        //lift.setTarget(0);
        //trajstate = TrajectoryState.Traj1;
        //drive.followTrajectorySequenceAsync(traj1);
        initTimer.reset();
        while (opModeIsActive() && !isStopRequested()) {

            switch (trajstate) {
                case Start:
                    if (initTimer.seconds() > 0.7) {
                        trajstate = TrajectoryState.Traj1;
                        drive.followTrajectorySequenceAsync(traj1);
                    }
                    break;
                case Traj1:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.Traj2;
                        drive.followTrajectorySequenceAsync(traj2);
                    }
                    break;

                case Traj2:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.Wait1;
                    }
                    break;
                case Wait1:
                    break;
                case Traj3:
                    if (!drive.isBusy()) {
                        claw.close();
                        trajstate = TrajectoryState.Wait2;
                        initTimer.reset();
                        firstRun = true;
                    }
                    break;
                case Wait2:
                    if (initTimer.seconds() > delay) {
                        drive.followTrajectorySequenceAsync(traj4);
                        trajstate = TrajectoryState.Traj4;
                    }
                    break;
                case Traj4:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.Wait3;
                    }
                    break;
                case Wait3:
                    break;
                case Traj5:
                    if (!drive.isBusy()) {
                        claw.close();
                        trajstate = TrajectoryState.Wait4;
                        initTimer.reset();
                        firstRun = true;
                    }
                    break;
                case Wait4:
                    if (initTimer.seconds() > delay) {
                        drive.followTrajectorySequenceAsync(traj6);
                        trajstate = TrajectoryState.Traj6;
                    }
                    break;
                case Traj6:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.Wait5;
                    }
                    break;
                case Wait5:
                    break;
                case Traj7:
                    if (!drive.isBusy()) {
                        trajstate = TrajectoryState.IDLE;
                    }
                        break;
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
                    switch (liftState) {
                        case START:
                            //if(trajstate == TrajectoryState.Traj2){
                            //if(Math.abs(claw.claw1.getPosition() - 0.87) < 0.01 && Math.abs(claw.claw2.getPosition()) < 0.01){
                            if (initTimer.seconds() > 0.5 && firstRun) {
                                lift.setTarget(HIGH_TARGET);
                                liftState = LiftState.EXTEND;
                                firstRun = false;
                            }

                            break;
                        case EXTEND:
                            if ((Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) && (trajstate == TrajectoryState.Wait1
                                    || trajstate == TrajectoryState.Wait3 || trajstate == TrajectoryState.Wait5 || trajstate == TrajectoryState.IDLE)) {
                                claw.open();
                                StackTimer.reset();
                                liftState = LiftState.STACK;
                            }
                            break;
                        case STACK:
                            if (StackTimer.seconds() >= STACK_TIME) { //may have to reduce STACK_TIME for optimization
                                lift.setTarget(ConeHeight);
                                ConeHeight -= coneDecrease;
                                liftState = LiftState.RETRACT;
                            /*
                            if (trajstate != TrajectoryState.Traj4 && trajstate != TrajectoryState.IDLE) {
                                drive.followTrajectorySequenceAsync(traj3);
                                trajstate = TrajectoryState.Traj3;
                            }
                             */
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
                            /*
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


                    lift.update();
                    drive.update();

                    Pose2d poseEstimate = drive.getPoseEstimate();

                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();


            }
        }


    }


