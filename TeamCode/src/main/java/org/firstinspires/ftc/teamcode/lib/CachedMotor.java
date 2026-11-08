package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * An extended motor class that utilizes more features than the
 * regular motor.
 */
public class CachedMotor {
    private double lastPower = Double.NaN;
    private int lastPos;
    private double lastVel;
    private final DcMotorEx motor;
    private double powerThreshold = 0.001;

    public CachedMotor(DcMotorEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public DcMotorEx getMotor(){
        return motor;
    }

    public void setPower(double power) {
        if (Double.isNaN(lastPower) || (Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
//        else { //setVelocity() doesn't get cached
//            RobotLog.a("Cached!");
//        }
    }

    public int getPosition() {
        lastPos = motor.getCurrentPosition();
        return(lastPos);
    }
    public int getLastPos(){
        return lastPos;
    }

    public double getVelocity() {
        lastVel = motor.getVelocity();
        return(lastVel);
    }
    public double getLastVel(){
        return lastVel;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
