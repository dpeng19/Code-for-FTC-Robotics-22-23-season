package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.jetbrains.annotations.NotNull;
@Disabled
@Autonomous(name = "RRTest")
public class RRTest extends LinearOpMode {
    private DcMotorEx slide;
    DistanceSensor dist;
    private final double INCHES_PER_REV = 4.4;
    private final double TICKS_PER_REV = 537.7;
    public static double TestHeight = 20;
    ElapsedTime timer = new ElapsedTime();
    public static double kP = 0.003, kI = 0, kD = 0;

    //set different positions
    public static double targetPosition = 0;
    public static double kV = 0, kA = 0, kStatic = 0;
    public static double kG = 0;
    PIDCoefficients coeffs= new PIDCoefficients(kP, kI, kD);
    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) -> kG);

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PhotonCore.enable();
        slide = hardwareMap.get(DcMotorEx.class, "lift");
        //slide.setDirection(DcMotorEx.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double Q = 1; // High values put more emphasis on the sensor.
        double R = 0.7; // High Values put more emphasis on regression.
        int N = 3; // The number of estimates in the past we perform regression on.
        KalmanFilter filter = new KalmanFilter(Q,R,N);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       // dist = hardwareMap.get(DistanceSensor.class, "distColor");
        //dist2 = hardwareMap.get(DistanceSensor.class, "distColor2");
        //color = hardwareMap.get(ColorSensor.class, "distColor");

        /*
        rightDist = hardwareMap.get(DistanceSensor.class, "distColor1");
        centerDist = hardwareMap.get(DistanceSensor.class, "distColor2");
        leftDist = hardwareMap.get(DistanceSensor.class, "distColor3");

         */
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));
        //drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(24)
                .forward(9.5)
                .splineTo(new Vector2d(-34, -12), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(45,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .strafeRight(10)
                //.UNSTABLE_addTemporalMarkerOffset(-1, () -> )
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-24, -9.5, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-24, -9.5, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-24, -8.5, Math.toRadians(90)))
                .waitSeconds(1)




/*
                .splineToConstantHeading(new Vector2d(-18, -15), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(45,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

 */


                .build();

        Pose2d startPose2 = new Pose2d(-35, -63.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose2);
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose2)
                .splineToConstantHeading(new Vector2d(-10, -61), Math.toRadians(0))
                .strafeLeft(39)
                .waitSeconds(1)
                .strafeLeft(4)
                .splineTo(new Vector2d(-34, -12), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(-55, -10, Math.toRadians(176)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-23.5, -11, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-55, -9.5, Math.toRadians(176)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-23.5, -10.5, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-55, -9, Math.toRadians(176)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-23.5, -10, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .build();
        Pose2d startPose3 = new Pose2d(-39, -63.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose3);
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(startPose3)
                .splineToConstantHeading(new Vector2d(-43, -61.5),Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .splineTo(new Vector2d(-59 ,-34), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .splineTo(new Vector2d(-59.5 ,-23), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .lineToLinearHeading(new Pose2d(-57, -10.8, Math.toRadians(180)))
                .waitSeconds(1)
                //.back(2)
                .lineToLinearHeading(new Pose2d(-54, -11.5, Math.toRadians(180)))
                .turn(Math.toRadians(125))
                .waitSeconds(1)
                .turn(Math.toRadians(-125))
                .lineToLinearHeading(new Pose2d(-56.5, -10.6, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-54, -11.5, Math.toRadians(180)))
                .turn(Math.toRadians(125))
                .waitSeconds(1)
                .turn(Math.toRadians(-125))
                .lineToLinearHeading(new Pose2d(-56.5, -10.4, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-54, -11.5, Math.toRadians(180)))
                .turn(Math.toRadians(125))
                .waitSeconds(1)
                .turn(Math.toRadians(-125))
                .lineToLinearHeading(new Pose2d(-56.5, -10.2, Math.toRadians(180)))
                .waitSeconds(1)
                //.turn(Math.toRadians(125))
                /*
                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(-45)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180)))

                 */
                .build();
        Pose2d startPose4 = new Pose2d(-39, -63.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose4);
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(startPose4)
                .splineToConstantHeading(new Vector2d(-11, -59), Math.toRadians(0))
                //.forward(24)

                .strafeLeft(35.5)
                .waitSeconds(1)

                .strafeLeft(4)
                .splineTo(new Vector2d(-34, -13), Math.toRadians(180))

                .lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-47, -10, Math.toRadians(-92)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-47, -10, Math.toRadians(-92)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-47, -10, Math.toRadians(-92)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .waitSeconds(1)
                        .build();

        Pose2d startPose5 = new Pose2d(-31, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose5);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(startPose5)
                .splineToConstantHeading(new Vector2d(-28, -60), Math.toRadians(0))

                .strafeRight(15)
                //.forward(48)
                //.strafeLeft(11)
                .forward(43)
                .splineToConstantHeading(new Vector2d(-23, -13), Math.toRadians(0))
                .strafeLeft(7)

                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(180)))
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-52, -13, Math.toRadians(-95)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-52, -13, Math.toRadians(-95)))
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-52, -13, Math.toRadians(-95)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-61, -12, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-52, -13, Math.toRadians(-95)))



                .waitSeconds(1)
                .strafeLeft(34)
                .turn(Math.toRadians(180))


                        .build();
        Pose2d startPose6 = new Pose2d(-32, -63.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose6);
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(startPose6)
                //.strafeRight(2)
                .setReversed(true)
                //.strafeRight(8)
                //.back(20)
                .splineToConstantHeading(new Vector2d(-12, -55.5), Math.toRadians(0))
                .setReversed(false)
                // .back(6)
                .strafeRight(32)
                .waitSeconds(1)
                //.splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(180))
                // .strafeRight(12)
                .strafeRight(6)
                .splineToConstantHeading(new Vector2d(-20, -13), Math.toRadians(180))
                //.setReversed(true)
                .forward(39)
                //.splineToConstantHeading(new Vector2d(-28, -60), Math.toRadians(0))
                //.strafeRight(15)
                //.forward(48)
                //.strafeLeft(11)
                //.forward(43)
                //.splineToConstantHeading(new Vector2d(-23, -12), Math.toRadians(0))
                //.strafeLeft(4)
                //.waitSeconds(1)
                //.lineToLinearHeading(new Pose2d(-56.5, -10, Math.toRadians(180)))
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-48, -13, Math.toRadians(-92)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-58,  -12.5, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-48, -13, Math.toRadians(-92)))
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-58, -12.5, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-48, -13, Math.toRadians(-92)))
                .waitSeconds(1)
               // .lineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(180)))
               // .waitSeconds(1)
               // .lineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(-90)))



               // .waitSeconds(1)
                .strafeLeft(34)
                .turn(Math.toRadians(180))



                .build();
        Pose2d startPose7 = new Pose2d(-32, -63.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose7);
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(startPose7)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.5, -55.5), Math.toRadians(0))
                .setReversed(false)
                .strafeRight(32.5)
                .waitSeconds(1)
                .strafeRight(5.5)
                .splineToConstantHeading(new Vector2d(-20, -13), Math.toRadians(180))
                .forward(40)
                .waitSeconds(1)
                .back(18)
                .splineToConstantHeading(new Vector2d(-38, -25), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(-59, -13), Math.toRadians(180))
                .waitSeconds(1)
                .back(18)
                .splineToConstantHeading(new Vector2d(-38, -25), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(-59, -13), Math.toRadians(180))
                .waitSeconds(1)
                .back(18)
                .splineToConstantHeading(new Vector2d(-38, -25), Math.toRadians(-90))
                .waitSeconds(1)
                        .build();


        Pose2d startPose8 = new Pose2d(38.5, -63.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose8);
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d(38.5, -63.5, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(28, -58.5), Math.toRadians(180))
                .forward(16.5)
                //.forward(9)
                //.splineToConstantHeading(new Vector2d(13.5, -43), Math.toRadians(90))
                .strafeRight(34.5)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(24, -13, Math.toRadians(90)), Math.toRadians(0))
                .strafeRight(12).build();
                /*
                .waitSeconds(1)

                .strafeRight(7)
                .splineToSplineHeading(new Pose2d(18, -12, Math.toRadians(0)), Math.toRadians(0))
                .forward(40)
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(47, -11.5, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(58,  -12, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47, -11.5, Math.toRadians(-90)))
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47, -11.5, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(47, -11.5, Math.toRadians(-90)))
                .waitSeconds(1)
                .strafeRight(34)
                .turn(Math.toRadians(180))












                .build();













        /*
        if(!isStopRequested()){
            drive.followTrajectorySequence(traj4);
            while(drive.isBusy()){
                telemetry.addData("distance", filter.estimate(dist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
            }
        }

         */
        waitForStart();
        if (isStopRequested()) return;
         drive.followTrajectorySequenceAsync(traj8);
        while(opModeIsActive()){
            drive.update();
           // telemetry.addData("distance", filter.estimate(dist.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("distance2", filter.estimate(dist2.getDistance(DistanceUnit.INCH)));
           // telemetry.update();
        }



    }
    public void extend(){
        controller.setTargetPosition(4000);
        while(Math.abs(slide.getCurrentPosition() - 4000) > 50) {
            double correction = controller.update(slide.getCurrentPosition());
            slide.setPower(correction);
        }
        timer.reset();
        while(true){
            if(timer.seconds() > 1)
                break;
        }
        controller.setTargetPosition(0);
        while(Math.abs(slide.getCurrentPosition() - 0) > 10) {
            double correction = controller.update(slide.getCurrentPosition());
            slide.setPower(correction);
        }
    }

}
