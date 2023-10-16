package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "ClawTest")
public class ClawTest extends LinearOpMode {
    private boolean openClaw = false;
    @Override
    public void runOpMode(){
        Claw claw = new Claw(hardwareMap);

        //claw1.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.x){
                claw.close();
            }
            if(gamepad1.y){
                claw.open();
            }




            /*
            if(gamepad1.x){
                claw.claw1.setPosition(0.5);
                claw.claw2.setPosition(0.5);
            }

             */




            //claw.claw1.setPosition(gamepad1.right_stick_y);
            //claw.claw2.setPosition(gamepad1.right_stick_y);




/*
        if(gamepad1.x){
             claw1.setPosition(0.5);
            claw2.setPosition(0.5);
        }

 */



            //claw1.setPosition(gamepad1.right_stick_x);
           // claw2.setPosition(gamepad1.left_stick_x);


/*
            if(gamepad1.b){
                if(!openClaw){
                    openClaw();
                    openClaw = true;
                }
                else{
                    closeClaw();
                    openClaw = false;
                }





            }

 */




            telemetry.addData("claw1", claw.claw1.getPosition());
            telemetry.addData("claw2", claw.claw2.getPosition());
            telemetry.update();
        }


    }
/*
    public void openClaw(){
        claw1.setPosition(0.15);
        claw2.setPosition(0.15);
    }
    public void closeClaw(){
        claw1.setPosition(0);
        claw2.setPosition(0);
    }

 */



}
