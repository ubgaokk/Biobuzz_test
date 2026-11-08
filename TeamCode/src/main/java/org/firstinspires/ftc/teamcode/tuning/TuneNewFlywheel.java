package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Voltage;
@Disabled

@Config
@TeleOp (name = "TuneNewFlywheel")
/**
 * runs two flywheel motors
 */
public class TuneNewFlywheel extends LinearOpMode {
    Flywheel f;
    Voltage v;
    Intake i;
    public static int targetVel = 0;
    public static double power1 = 0;
    public static double power2 = 0;
    public void runOpMode(){
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        f = new Flywheel(hardwareMap);
        v = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
        i = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            double k = 0;
            if (gamepad1.rightBumperWasPressed()){
                targetVel += 100;
            }
            if (gamepad1.leftBumperWasPressed()){
//                power2 += 0.01;
                targetVel -= 100;
            }
            if (gamepad1.dpadUpWasPressed()){
                i.shoot();
            }
            if (gamepad1.dpadDownWasPressed()){
                i.stop();
            }
            if (gamepad1.b){
                targetVel = 0;
                power1 = 0;
                power2 = 0;
                f.FLYWHEEL_MASTER.setPower(0);
                f.FLYWHEEL2.setPower(0);
            }
            else {
//                f.FLYWHEEL_MASTER.setPower(power1);
//                f.FLYWHEEL2.setPower(power2);
                f.spinTo(targetVel);
            }
            telemetry.addData("power 1", f.FLYWHEEL_MASTER.getPower());
            telemetry.addData("power 2", f.FLYWHEEL2.getPower());
            telemetry.addData("target v", targetVel);
            telemetry.addData("motor v", f.getVel());
            telemetry.addData("motor volts", k);
            telemetry.update();
        }
    }
}
