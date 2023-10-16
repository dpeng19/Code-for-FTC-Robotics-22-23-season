package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Config
@Autonomous(name = "SlideTest")
public class SlideTest extends LinearOpMode {
    private DcMotorEx slide;
    private final double INCHES_PER_REV = 4.40945;
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;
    public static double kP = 0, kI = 0, kD = 0;

    //set different positions
    public static double targetPosition = 32;
    public static double kV = 0, kA = 0, kStatic = 0;
    public static double kG = 0;
    PIDCoefficients coeffs= new PIDCoefficients(kP, kI, kD);
    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x,v) -> kG);

    private final double maxVel = 21, maxAccel = 40;

    @Override

    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = hardwareMap.get(DcMotorEx.class, "lift");

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        NanoClock clock = NanoClock.system();
        waitForStart();

        if (isStopRequested()) return;
        boolean movingUp= true;
        MotionProfile activeProfile = generateProfile(true, targetPosition);
        double profileStart = clock.seconds();
        while(!isStopRequested()){
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {

                movingUp = !movingUp;
                activeProfile = generateProfile(movingUp, targetPosition);
                profileStart = clock.seconds();

            }
            MotionState state = activeProfile.get(profileTime);
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getA());
            double correction = controller.update(slide.getCurrentPosition()/TICKS_PER_INCH);
            slide.setPower(correction);
            telemetry.addData("correction", correction);
            telemetry.addData("measuredPosition", slide.getCurrentPosition() / TICKS_PER_INCH);
            telemetry.addData("targetPosition", controller.getTargetPosition());
            telemetry.addData("measuredVelocity", slide.getVelocity() / TICKS_PER_INCH);
            telemetry.addData("targetVelocity", controller.getTargetVelocity());
            telemetry.addData("targetAccel", controller.getTargetAcceleration());
            telemetry.update();
        }

    }
    public MotionProfile generateProfile(boolean movingUp, double dist) {
        MotionState start = new MotionState(movingUp ? 0 : dist, 0, 0, 0);
        MotionState goal = new MotionState(movingUp ? dist : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, maxVel, maxAccel);
    }









}
