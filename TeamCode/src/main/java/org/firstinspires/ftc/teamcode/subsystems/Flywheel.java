package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedMotor;
import org.firstinspires.ftc.teamcode.lib.FeedforwardController;
import org.firstinspires.ftc.teamcode.lib.PIDController;

//two-motor flywheel
public class Flywheel {
    //pid methods
    public CachedMotor FLYWHEEL_MASTER;
    public CachedMotor FLYWHEEL2;
    double targetVel;
    PIDController pid;
    FeedforwardController f;
    VoltageSensor v;
    public static double volts;
    double radialFF = 0;
    //power
    public Flywheel(HardwareMap hardwareMap){
        FLYWHEEL_MASTER = new CachedMotor(hardwareMap.get(DcMotorEx.class,"flywheel"));
        FLYWHEEL_MASTER.setDirection(DcMotorSimple.Direction.FORWARD);
        FLYWHEEL_MASTER.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL_MASTER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL_MASTER.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FLYWHEEL2 = new CachedMotor(hardwareMap.get(DcMotorEx.class,"flywheel2"));
        FLYWHEEL2.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        v = hardwareMap.get(VoltageSensor.class, "Control Hub");
        pid = new PIDController(Robot.Constants.kP,Robot.Constants.kI,Robot.Constants.kD);
        f = new FeedforwardController(Robot.Constants.kS,Robot.Constants.kV);
    }

    public void setPower(double power){
        FLYWHEEL_MASTER.setPower(power);
        FLYWHEEL2.setPower(power);
    }
    public void setRadialFeedforward(double radialSpeed) {
        radialFF = Robot.Constants.kRadial * radialSpeed;
    }

    /**
     * @param vel Velocity the flywheel will spin at
     */
    public double spinTo(double vel) {
//        if (configvalues.p != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p || configvalues.f != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f)
//            FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(configvalues.p, configvalues.i, configvalues.d, configvalues.f));
        targetVel = vel;
        double curVel = getVel();
        //pidf bangbang: go as fast as possible when spinning up then slow down for the last hundred ticks/s (doesnt work well)
        if (/*Math.abs(targetVel-curVel) > 300*/ true) {
            double pdds = pid.calculate(curVel, targetVel);
//        FLYWHEEL.getMotor().setVelocity(targetVel);
            volts = v.getVoltage();
            double power = (pdds + f.calculate(targetVel) + radialFF) / volts;
            setPower(power);
            return power * volts;
        }
        else{ //it doesn't go here
            double power = Math.signum(targetVel-curVel);
            setPower(power);
            return power;
        }
    }

    // Return flywheel current velocity
    public double getVel(){
        return FLYWHEEL_MASTER./*getMotor().*/getVelocity();
    }
}
