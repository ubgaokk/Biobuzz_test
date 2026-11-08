package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedMotor;
import org.firstinspires.ftc.teamcode.lib.FeedforwardController;
import org.firstinspires.ftc.teamcode.lib.PIDController;

@Config
//two-motor flywheel
public class Flywheel {
    public DcMotorEx FLYWHEEL_MASTER;
    public DcMotorEx FLYWHEEL2;
    double targetVel;
    PIDController pid;
    FeedforwardController f;
    VoltageSensor v;
    double volts;
    public static double kP = 0, kI = 0, kD = 0, kS = 1.85, kV = 0.00564;
    public Flywheel(HardwareMap hardwareMap){
        FLYWHEEL_MASTER = hardwareMap.get(DcMotorEx.class,"flywheel");
        FLYWHEEL_MASTER.setDirection(DcMotorSimple.Direction.FORWARD);
        FLYWHEEL_MASTER.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL_MASTER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL_MASTER.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FLYWHEEL2 = hardwareMap.get(DcMotorEx.class,"flywheel2");
        FLYWHEEL2.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        v = hardwareMap.get(VoltageSensor.class, "Control Hub");
        pid = new PIDController(kP,kI,kD);
        f = new FeedforwardController(kS,kV);
    }

    public void setPower(double power){
        FLYWHEEL_MASTER.setPower(power);
        FLYWHEEL2.setPower(power);
    }

    /**
     * @param vel Velocity the flywheel will spin at
     */
    public double spinTo(double vel) {
        targetVel = vel;
        double curVel = getVel();
        double pidVolts = pid.calculate(curVel, targetVel);
        volts = v.getVoltage();
        double power = (pidVolts + f.calculate(targetVel)) / volts;
        setPower(power);
        return power * volts;
    }

    // Return flywheel current velocity
    public double getVel(){
        return FLYWHEEL_MASTER.getVelocity();
    }
}
