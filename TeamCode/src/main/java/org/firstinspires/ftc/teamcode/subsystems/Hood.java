package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

public class Hood {
    public CachedServo HOOD;

    public Hood(HardwareMap hardwareMap){
//        HOOD = hardwareMap.get(Servo.class, "hood");
        HOOD = new CachedServo(hardwareMap.get(Servo.class, "hood"));
        HOOD.setDirection(Servo.Direction.FORWARD);
        HOOD.scaleRange(0,1); //Robot.Constants.hoodScale0, Robot.Constants.hoodScale1); // left = low, right = high
    }

    public void setPosition( double pos){
        HOOD.setPosition(clip(scale(pos)));
//        RobotLog.a("" +clip(scale(pos)));
    }
    //Sets hood position to a certain angle (relative to ground/horizontal)
    public void turnToAngle(double angle){
        setPosition((angle - Robot.Constants.hoodLowAngle) / (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle));
    }

    // Returns angle of hood (radians)
    public double getAngle(){
        return unscale(HOOD.getPosition()) * (Robot.Constants.hoodHighAngle - Robot.Constants.hoodLowAngle) + Robot.Constants.hoodLowAngle;
    }

    // Hood will not move if set past its limit, so this function returns the closest position that is still in range
    public double clip(double pos){
        return Range.clip(pos,Robot.Constants.hoodClip0,Robot.Constants.hoodClip1);
    }
    public double scale (double pos){
        return Range.scale(pos, 0, 1, Robot.Constants.hoodScale0, Robot.Constants.hoodScale1);
    }
    public double unscale(double pos){
        return Range.scale(pos, Robot.Constants.hoodScale0, Robot.Constants.hoodScale1, 0, 1);
    }
}
