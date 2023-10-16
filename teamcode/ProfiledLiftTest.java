package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.drive.VoltageCompensatedFF.tunekA;
import static org.firstinspires.ftc.teamcode.drive.VoltageCompensatedFF.tunekStatic;
import static org.firstinspires.ftc.teamcode.drive.VoltageCompensatedFF.tunekV;
import static org.firstinspires.ftc.teamcode.subsystems.Lift.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.drive.opmode.DriveVelocityPIDTuner;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class ProfiledLiftTest extends LinearOpMode {
    public static double dist = 0;
    //public static double kVUp = 0, kAUp = 0, kStaticUp = 0;
    //public static double kVDown = 0, kADown = 0, kStaticDown = 0;
    final double INCHES_PER_REV = 4.40945;
    final double TICKS_PER_REV = 384.5;
    final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;
    public DcMotorEx slide;
    //define PIDF variables and coefficients
    public static double kPUp = 0, kIUp = 0, kDUp = 0;
    public static double kVUp = 0.0003, kAUp = 0, kStaticUp = 0;
    public static double kGUp = 0.2;

    public static double kPDown = 0, kIDown = 0, kDDown = 0;
    public static double kVDown = 0.0003, kADown = 0, kStaticDown = 0;
    public static double kGDown = 0.2;
    PIDCoefficients coeffsUp;
    PIDFController controllerUp;

    PIDCoefficients coeffsDown;
    PIDFController controllerDown;

    private boolean wait = false;
    ElapsedTime timer =new ElapsedTime();


    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : dist * 384.5/4.40945, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? dist * 384.5/4.40945 : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 28 * 384.5/4.40945, 25 * 384.5/4.40945);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "lift");
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        coeffsUp = new PIDCoefficients(kPUp, kIUp, kDUp);
        controllerUp = new PIDFController(coeffsUp, kVUp, kAUp, kStaticUp, (x, v) -> kGUp);

        coeffsDown = new PIDCoefficients(kPDown, kIDown, kDDown);
        controllerDown = new PIDFController(coeffsDown, kVDown, kADown, kStaticDown, (x, v) -> kGDown);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {


            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStart;

            if (!wait && profileTime > activeProfile.duration()) {
                // generate a new profile
                wait = true;
                /*
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();

                 */
                timer.reset();
            }
            if(timer.seconds() > 2 && wait){
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();
                profileTime = clock.seconds() - profileStart;
                wait = false;
            }

            if(!wait) {
                MotionState state = activeProfile.get(profileTime);
                double correction;
                if (movingForwards) {
                    controllerUp.setTargetPosition(state.getX());
                    controllerUp.setTargetVelocity(state.getV());
                    controllerUp.setTargetAcceleration(state.getA());
                    correction = controllerUp.update(slide.getCurrentPosition(), slide.getVelocity());
                    slide.setPower(correction);
                } else {
                    controllerDown.setTargetPosition(state.getX());
                    controllerDown.setTargetVelocity(state.getV());
                    controllerDown.setTargetAcceleration(state.getA() + 9.8);
                    correction = controllerDown.update(slide.getCurrentPosition(), slide.getVelocity());
                    slide.setPower(correction);
                }


                // update telemetry
                telemetry.addData("targetVelocity", state.getV());
                telemetry.addData("measuredVelocity", slide.getVelocity());
                telemetry.addData("correction", correction);
            }
            else {
                telemetry.addData("targetVelocity", 0);
                telemetry.addData("measuredVelocity", slide.getVelocity());
            }
            //coeffs = new PIDCoefficients(kP, kI, kD);
            //controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) -> kG);

            //lift.update(state);


           // telemetry.addData("measuredVelocity", lift.slide.getVelocity());
            //telemetry.addData("targetVelocity", motionState.getV());




            telemetry.update();
        }
    }
}
