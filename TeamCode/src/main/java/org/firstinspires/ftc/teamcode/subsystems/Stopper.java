package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

public class Stopper {
   public CachedServo STOPPER;

    public Stopper(HardwareMap hardwareMap){
        STOPPER = new CachedServo(hardwareMap.get(Servo.class, "stopper"));
        STOPPER.setDirection(Servo.Direction.REVERSE);
        STOPPER.scaleRange(0, 1); //open = , close =
    }
    public void open(){
        STOPPER.setPosition(Robot.Constants.stopperScale0);
    }
    public void close(){
        STOPPER.setPosition(Robot.Constants.stopperScale1);
    }

}
