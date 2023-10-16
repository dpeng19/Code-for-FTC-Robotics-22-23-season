package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.HEADING_PID;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.TRANSLATIONAL_PID;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous (name = "LinearHeadingTest")
public class LinearHeadingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(180)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)))

                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequenceAsync(traj);
        while(opModeIsActive()){
            drive.update();
            telemetry.addData("translation_kD", TRANSLATIONAL_PID.kD);
            telemetry.addData("heading_kD", HEADING_PID.kD);
            telemetry.update();
        }
    }
}
