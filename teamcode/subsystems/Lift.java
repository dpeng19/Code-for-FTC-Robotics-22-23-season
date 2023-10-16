package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Lift {
    public DcMotorEx slide;
    //define PIDF variables and coefficients
    private double kP = 0.004, kI = 0.00005, kD = 0;
    private double kV = 0, kA = 0, kStatic = 0;
    private double kG = 0;
    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) -> kG);
    public static double liftOffset = 0;

    public Lift(@NonNull HardwareMap hwMap){
        slide = hwMap.get(DcMotorEx.class, "lift");
        //slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //maybe
        /*
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

         */
    }
    public void setTarget(double targetPosition){

        controller.setTargetPosition(targetPosition);
    }
    public double getTarget(){
        return controller.getTargetPosition();
    }
    public void update(){
        slide.setPower(controller.update(slide.getCurrentPosition() + liftOffset));
    }
    public void update(MotionState state){
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
        slide.setPower(controller.update(slide.getCurrentPosition()));

    }


}
