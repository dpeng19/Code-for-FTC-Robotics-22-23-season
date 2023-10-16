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
@Autonomous(name = "LiftTest")
public class LiftTest extends LinearOpMode {



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
   // public static PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x,v) -> kG);
    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) ->kG);

    public static double kV2 = 0, kA2 = 0, kStatic2 = 0;
    public static double kG2 = 0;
    public static double kP2 = 0, kI2 = 0, kD2 = 0;
    PIDCoefficients coeffs2= new PIDCoefficients(kP2, kI2, kD2);
    PIDFController controller2 = new PIDFController(coeffs2, kV2, kA2, kStatic2, (x, v) ->kG2);

    ElapsedTime timer = new ElapsedTime();
    double correction;



    public static double maxVelUp = 21, maxAccelUp = 50;
    public static double maxVelDown = 20, maxAccelDown = 20;
    private boolean firstTime = true;

    /*
    private double mV, mA = 0;
    private double lastTime = 0;
    private double lastPos, lastVel = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    private boolean reset = false;
    private boolean generateProfile = true;

    private final double LOW_TARGET = 1600/INCHES_PER_REV;
    private final double MED_TARGET = 3000/INCHES_PER_REV;
    private final double HIGH_TARGET = 4400/INCHES_PER_REV;
    */



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
       // timer.reset();
        while(!isStopRequested()){
            if(timer.seconds() < 5)
                continue;
            if(timer.seconds() > 5 && firstTime){
                movingUp= true;
                activeProfile = generateProfile(true, targetPosition);
                profileStart = clock.seconds();
                firstTime = false;
            }
/*
            if(targetPosition != 0 && !reset){
                timer2.reset();
                reset = true;
            }
            controller.setTargetPosition(targetPosition);
            double correction = controller.update(slide.getCurrentPosition());
            slide.setPower(correction);
            double currPos = slide.getCurrentPosition();
            double currVel = slide.getVelocity();
            double currTime = timer.seconds();
            mV = Math.max(mV, (currPos - lastPos) / (currTime - lastTime));
            mA = Math.max(mA, (currVel - lastVel) / (currTime - lastTime));
            //telemetry.addData("correction", correction);
            telemetry.addData("position", slide.getCurrentPosition());
            telemetry.addData("velocity", slide.getVelocity());
            telemetry.addData("acceleration", (currVel - lastVel) / (currTime - lastTime));
            telemetry.addData("target position", targetPosition);
            telemetry.addData("maxVel", mV);
            telemetry.addData("maxAccel", mA);
            telemetry.addData("timer", timer2.seconds());
            telemetry.update();
            lastTime = currTime;
            lastPos = currPos;
            lastVel = currVel;
*/




            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {
                //timer.reset();
               // generateProfile = false;
                //if(timer.seconds() > 0.5){
                    // generate a new profile
                    movingUp = !movingUp;
                    activeProfile = generateProfile(movingUp, targetPosition);
                    profileStart = clock.seconds();
                    //generateProfile = true;
                //}
            }
            telemetry.addData("duration", activeProfile.duration());
            //if(generateProfile){
                MotionState state = activeProfile.get(profileTime);
                if(movingUp){
                    controller.setTargetPosition(state.getX());
                    controller.setTargetVelocity(state.getV());
                    controller.setTargetAcceleration(state.getA());
                    correction = controller.update(slide.getCurrentPosition());
                    slide.setPower(correction);
                    telemetry.addData("correction", correction);
                    telemetry.addData("targetPosition", controller.getTargetPosition());
                    telemetry.addData("measuredPosition", slide.getCurrentPosition() / TICKS_PER_INCH);
                    telemetry.addData("measuredVelocity", slide.getVelocity() / TICKS_PER_INCH);
                    telemetry.addData("targetVelocity", controller.getTargetVelocity());
                    telemetry.addData("targetAccel", controller.getTargetAcceleration());
                }
                else{
                    controller2.setTargetPosition(state.getX());
                    controller2.setTargetVelocity(state.getV());
                    controller2.setTargetAcceleration(state.getA());
                    correction = controller2.update(slide.getCurrentPosition());
                    slide.setPower(correction);
                    telemetry.addData("correction", correction);
                    telemetry.addData("targetPosition", controller2.getTargetPosition());
                    telemetry.addData("measuredPosition", slide.getCurrentPosition() / TICKS_PER_INCH);
                    telemetry.addData("measuredVelocity", slide.getVelocity() / TICKS_PER_INCH);
                    telemetry.addData("targetVelocity", controller2.getTargetVelocity());
                    telemetry.addData("targetAccel", controller2.getTargetAcceleration());
                }

                //slide.setPower(controller.update(slide.getCurrentPosition()/TICKS_PER_INCH, slide.getVelocity()/TICKS_PER_INCH));

                telemetry.update();
            //}






        }

    }

    public MotionProfile generateProfile(boolean movingUp, double dist) {
        MotionState start = new MotionState(movingUp ? 0 : dist, 0, 0, 0);
        MotionState goal = new MotionState(movingUp ? dist : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, movingUp? maxVelUp: maxVelDown, movingUp ? maxAccelUp: maxAccelDown);

    }










}
