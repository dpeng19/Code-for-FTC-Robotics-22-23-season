package org.firstinspires.ftc.teamcode.drive;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class VoltageCompensatedFF {
    //public static double adjustedkV, adjustedkA, adjustedkStatic;
    public static double coeff;
    public static double batteryUpdate;
    //public static VoltageSensor batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
    public static double getkV(@NonNull HardwareMap hwMap){
        VoltageSensor batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
        coeff = batteryVoltageSensor.getVoltage();
        //adjustedkV = kV * 12 / coeff;
        //return adjustedkV;
        return kV * 12 / coeff;
    }
    public static double getkA(@NonNull HardwareMap hwMap){
        //VoltageSensor batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
        //adjustedkA = kA * 12 / coeff;
        //return adjustedkA;
        return kA * 12 / coeff;
    }
    public static double getkStatic(@NonNull HardwareMap hwMap){
        //VoltageSensor batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
        //adjustedkStatic = kStatic * 12 / coeff;
        //return adjustedkStatic;
        return kStatic * 12 / coeff;
    }
    public static double updatekV(){
        return kV * 12/coeff;
    }
    public static double updatekA()
    {
        return kA * 12/coeff;
    }
    public static double updatekStatic()
    {
        return kStatic * 12/coeff;
    }
    public static double tunekV(@NonNull HardwareMap hwMap){
        batteryUpdate = hwMap.voltageSensor.iterator().next().getVoltage();
        return kV * 12/batteryUpdate;
    }
    public static double tunekA(HardwareMap hwMap){
        return kA * 12/batteryUpdate;
    }
    public static double tunekStatic(HardwareMap hwMap){
        return kStatic * 12/batteryUpdate;
    }
}

