package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedServo;

//two CR axons and 1 analog input
public class Turret {
    public CachedServo turret;
    public CachedServo turret2; //make this cached later
    public static double last_pos = 0;

    public Turret(HardwareMap hardwareMap){
        turret = new CachedServo(hardwareMap.get(Servo.class,"turret"));
//        turret.setDirection(Servo.Direction.REVERSE);
        turret2 = new CachedServo(hardwareMap.get(Servo.class, "turret2"));
        turret.scaleRange(0,1); // 0 = +90 deg, 1 = -330 deg; DOUBLE CHECK
        turret2.scaleRange(0,1);
//        turret.getServo().getController().pwmDisable();
    }

    // Turns turret to the robot-relative angle in radians
    public void turnToRobotAngle(double angle) {
        angle = AngleUnit.normalizeRadians(angle- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset;
        double current_pos = constrain(Range.scale(angle,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle,Robot.Constants.turretScale0,Robot.Constants.turretScale1));
        turret.setPosition(current_pos);
        turret2.setPosition(current_pos);
//        if (Math.abs(current_pos - last_pos) > 0.05)
//            turret2.setPower(Math.signum(current_pos-last_pos)); //check direction of this before
//        last_pos = current_pos;
    }

    // Gives turret's robot-relative angle (in radians)
    public double getTurretRobotAngle() {
        return Range.scale(turret.getPosition(), Robot.Constants.turretScale0,Robot.Constants.turretScale1,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle);
    }

    // Gives turret's robot-relative pose based on its position and current heading
    public Pose getPoseRobotTurret() {
        return new Pose(Robot.Constants.turretPos.position.x,Robot.Constants.turretPos.position.y,getTurretRobotAngle());
    }

    // Function returns the closest position that is still in range (turret will not move if not in range)
    public double constrain(double ogPos){
        return Range.clip(ogPos,Robot.Constants.turretClip0,Robot.Constants.turretClip1);
    }
}
