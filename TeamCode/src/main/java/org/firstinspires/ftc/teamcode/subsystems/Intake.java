package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.CachedMotor;

public class Intake {
    //power
    public CachedMotor INTAKE;
    public CachedMotor TRANSFER;
    double power1 = 0;
    double power2 = 0;
    boolean paused = false;
    double voltageComp = 1;
    //power
    public Intake(HardwareMap hardwareMap){
        INTAKE = new CachedMotor(hardwareMap.get(DcMotorEx.class, "intake"));
        INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
        INTAKE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RobotLog.a(INTAKE.getMotor().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p + ", " + INTAKE.getMotor().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i + ", " + INTAKE.getMotor().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d + ", " + INTAKE.getMotor().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE.getMotor().setVelocityPIDFCoefficients(Robot.Constants.iP,Robot.Constants.iI,Robot.Constants.iD,Robot.Constants.iF);
        TRANSFER = new CachedMotor(hardwareMap.get(DcMotorEx.class, "transfer"));
        TRANSFER.setDirection(DcMotorSimple.Direction.REVERSE);
        TRANSFER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TRANSFER.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TRANSFER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TRANSFER.getMotor().setVelocityPIDFCoefficients(Robot.Constants.tP,Robot.Constants.tI,Robot.Constants.tD,Robot.Constants.tF);
    }
    public void telemetry(Telemetry telemetry){
        telemetry.addData("intake", INTAKE.getVelocity());
        telemetry.addData("transfer", TRANSFER.getVelocity());
    }
    // Modes
    public void intake(){
        if (!paused) {INTAKE.setPower(1* Robot.Constants.intakePower); TRANSFER.setPower(1*Robot.Constants.throughPower /*1* Robot.Constants.intakePower*/);}
        power1 = 1*Robot.Constants.intakePower;
        power2 = 1*Robot.Constants.throughPower;
    }
    public void slowIntake(){
        if (!paused) {INTAKE.getMotor().setVelocity(300); TRANSFER.getMotor().setVelocity(1400 /*1* Robot.Constants.slowIntakePower*/);}
//        power1 = 1*Robot.Constants.slowIntakePower;
//        power2 = 0;
    }
//    public void slowIntake(){
//        if (!paused) {INTAKE.setPower(1*Robot.Constants.slowIntakePower); TRANSFER.setPower(0 /*1* Robot.Constants.slowIntakePower*/);}
//        power1 = 1*Robot.Constants.slowIntakePower;
//        power2 = 0;
//    }
    public void shoot(){
        if (!paused) {INTAKE.getMotor().setVelocity(Robot.Positions.shootPower*1900); TRANSFER.getMotor().setVelocity(Robot.Positions.transferPower*1900); /*INTAKE.setPower(1* Robot.Positions.shootPower+Robot.Positions.shootPowerOffset); TRANSFER.setPower(1* Robot.Positions.transferPower+Robot.Positions.transferPowerOffset);*/}
        power1 = 1*Robot.Positions.shootPower;
        power2 = 1*Robot.Positions.transferPower;
    }
    public void outtake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); TRANSFER.setPower(-1* Robot.Constants.outtakePower);}
        power1 = -1*Robot.Constants.outtakePower;
        power2 = -1*Robot.Constants.outtakePower;
    }
    public void strongOuttake(){
        if (!paused) {INTAKE.setPower(-1); TRANSFER.setPower(-1);}
        power1 = -1;
        power2 = -1;
    }
    public void stop(){
        if (!paused) {INTAKE.setPower(0); INTAKE.getMotor().setVelocity(0); TRANSFER.setPower(0);TRANSFER.getMotor().setVelocity(0); }
        power1 = 0;
        power2 = 0;
    }
    // Start and stop
    public void pause(){
        INTAKE.setPower(0);
        TRANSFER.setPower(0);
        paused = true;
    }
    public void shortOuttake(){
        if (!paused) {INTAKE.setPower(-1*Robot.Constants.outtakePower); TRANSFER.setPower(-1*Robot.Constants.outtakePower);}
        paused = true;
    }
    public void proceed() {
        INTAKE.setPower(power1 * voltageComp);
        TRANSFER.setPower(power2 * voltageComp);
        paused = false;
    }
//    public void updateComp(){
//        voltageComp = 14.0/Flywheel.volts;
////        if (voltageComp %0.1==0){
//            RobotLog.d("comp"+voltageComp);
//            RobotLog.d("volts"+Flywheel.volts);
////        }
//    }
}