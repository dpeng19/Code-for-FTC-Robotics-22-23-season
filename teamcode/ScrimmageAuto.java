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
@Autonomous(name = "ScrimmageAuto")
public class ScrimmageAuto extends LinearOpMode {


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
    private double ConeHeight = 650;
    private final double coneDecrease = 1.25 * TICKS_PER_INCH;


    enum TrajectoryState {
        Start,
        Traj1,
        Traj2,
        Wait1,
        Traj3,
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
                .splineToConstantHeading(new Vector2d(-10.9, -61), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                //.forward(24)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .strafeLeft(39.2, SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .turn(Math.toRadians(88.5))
                .back(20)
                //.back(2)
                .build();




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
                    if(initTimer.seconds() > 0.7){
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
                    break;
                case IDLE:
                    break;
            }
            telemetry.addData("initTimer", initTimer.seconds());
            telemetry.addData("claw1pos", claw.claw1.getPosition());
            telemetry.addData("claw2pos", claw.claw2.getPosition());
            telemetry.addData("slidepos", lift.slide.getCurrentPosition());
            /*
            if(initTimer.seconds() > 0.5 && firstRun){
                lift.setTarget(300);
                firstRun = false;
            }

             */
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
                    if ((Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) && trajstate == TrajectoryState.Wait1) {
                        claw.open();
                        StackTimer.reset();
                        liftState = LiftState.STACK;
                    }
                    break;
                case STACK:
                    if (StackTimer.seconds() >= STACK_TIME) {
                        lift.setTarget(-15);
                        liftState = LiftState.RETRACT;
                        /*
                        lift.setTarget(ConeHeight);
                        ConeHeight -= coneDecrease;
                        liftState = LiftState.RETRACT;
                        trajstate = TrajectoryState.Traj3;
                        drive.followTrajectorySequenceAsync(traj3);

                         */

                    }
                    break;
                case RETRACT:
                    if (Math.abs(lift.slide.getCurrentPosition() - lift.getTarget()) < 50 || Math.abs(lift.slide.getCurrentPosition() - lastPos) <= 1) {
                        liftState = LiftState.START;
                        trajstate = TrajectoryState.Traj3;
                        drive.followTrajectorySequenceAsync(traj3);
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
