package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Disabled
@TeleOp
public class TuneFeedforwardNew extends OpMode {
    DcMotorEx FLYWHEEL;
    DcMotorEx FLYWHEEL2;
    double POWER_PER_SEC = 0.1;
    double POWER_MAX = 1;
    ElapsedTime t;
    VoltageSensor v;

    public void init(){
        FLYWHEEL = hardwareMap.get(DcMotorEx.class, "flywheel");
        FLYWHEEL.setDirection(DcMotorSimple.Direction.FORWARD);
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FLYWHEEL2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        FLYWHEEL2.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLYWHEEL2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        v = hardwareMap.get(VoltageSensor.class,"Control Hub");
//        FLYWHEEL.setVelocityPIDFCoefficients();
        t = new ElapsedTime();
        telemetry.update();
    }
    public void start(){
        t.reset();
    }
    public void loop(){
        double time = t.seconds();
        double power = power(time);
        FLYWHEEL.setPower(power);
        FLYWHEEL2.setPower(power);
        double vel = FLYWHEEL.getVelocity();
        double bv = v.getVoltage();
        double vapp = power * bv;
        RobotLog.dd(""+vel,""+vapp); //graph vapp as y vs. vel as x
        telemetry.addData("vel", vel);
        telemetry.addData("seconds", time);
        telemetry.addData("power", power);
        telemetry.addData("voltage applied", vapp);
        telemetry.addData("battery voltage", bv);
        telemetry.update();
    }
    public double power(double seconds){
        return Math.min(POWER_PER_SEC * seconds, POWER_MAX);
    }

}

