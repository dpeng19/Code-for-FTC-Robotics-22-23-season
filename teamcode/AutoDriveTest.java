package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@TeleOp(name = "AutoDriveTest")
public class AutoDriveTest extends LinearOpMode {
    private boolean pGA1X = false;
    private boolean score = true;
    enum DriveState{
        MANUAL,
        AUTO
    }
    DriveState driveState = DriveState.MANUAL;
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TrajectorySequence lowScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .setReversed(true)
                .back(12)
                .splineToConstantHeading(new Vector2d(-26.5, -39.5), Math.toRadians(-180))
                .build();
        TrajectorySequence lowBack =  drive.trajectorySequenceBuilder(new Pose2d(-24, -35.5, Math.toRadians(-90)))
                .strafeLeft(4)
                .splineToConstantHeading(new Vector2d(-12, -57), Math.toRadians(-90))
                .build();
        //MED-THERE
        TrajectorySequence medScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .back(12)
                .lineToLinearHeading(new Pose2d(-11.5, -25, Math.toRadians(-180)))
                .build();

        //MED-BACK
        TrajectorySequence medBack = drive.trajectorySequenceBuilder(new Pose2d(-11, -25, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-13, -34,  Math.toRadians(-94)))
                .forward(22.5)
                .build();

        //HIGH-THERE
        TrajectorySequence highScore = drive.trajectorySequenceBuilder(new Pose2d(-12, -57, Math.toRadians(-90)))
                .back(18)
                .lineToLinearHeading(new Pose2d(-13, -26, Math.toRadians(0)))
                .build();


        //HIGH-BACK
        TrajectorySequence highBack = drive.trajectorySequenceBuilder(new Pose2d(-12.5, -24, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-10, -34,  Math.toRadians(-94)))
                .forward(21)
                .build();
        //HIGH-THERE-ALT
        TrajectorySequence highScoreAlt = drive.trajectorySequenceBuilder(new Pose2d(-12, -58, Math.toRadians(0)))
                .strafeLeft(34.5)
                .build();


        //HIGH-BACK-ALT
        TrajectorySequence highBackAlt = drive.trajectorySequenceBuilder(new Pose2d(-12, -23.5, Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-13.5, -28), Math.toRadians(-90))
                .strafeRight(30)
                .build();

        waitForStart();
        while(opModeIsActive()){
            drive.update();
            switch(driveState){
                case MANUAL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    if(gamepad1.x){
                        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if(score) {
                            drive.setPoseEstimate(new Pose2d(-12, -58, Math.toRadians(0)));
                            drive.followTrajectorySequenceAsync(highScoreAlt);
                        }
                        else{
                            drive.setPoseEstimate(new Pose2d(-12, -23.5, Math.toRadians(0)));
                            drive.followTrajectorySequenceAsync(highBackAlt);
                        }
                        score = !score;
                        driveState = DriveState.AUTO;
                    }


                break;
                case AUTO:
                    if(gamepad1.y)
                    {
                        drive.breakFollowing();
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        driveState = DriveState.MANUAL;

                    }
                    if(!drive.isBusy()){
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        driveState = DriveState.MANUAL;
                    }
                    break;


            }



        }

    }
}
