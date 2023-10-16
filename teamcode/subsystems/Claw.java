package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw1, claw2;
    public Claw(@NonNull HardwareMap hwMap){
        claw1 = hwMap.get(Servo.class, "claw1");
        claw2 = hwMap.get(Servo.class, "claw2");
        //for (LynxModule module : hwMap.getAll(LynxModule.class))
            //module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
    public void open(){
        claw1.setPosition(0.4); //maybe 0.35
        claw2.setPosition(0.77);
    }
    public void close(){
        claw1.setPosition(1.0);
        claw2.setPosition(0.1); //maybe 0.3
    }




}
