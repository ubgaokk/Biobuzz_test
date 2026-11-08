package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

//@Disabled

@TeleOp
public class newTurretTest extends LinearOpMode {
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    Turret turret;//    private DigitalChannel LED_DigitalChannel;

    @Override
    public void runOpMode() throws InterruptedException {
 //        initialization();
        turret = new Turret(hardwareMap);
        waitForStart();
        double turretPos = 0.75;
        while (opModeIsActive()) {
            // Run flywheel based on calculated velocity
//            flywheelVelocity = calcShooterVel(telemetry, distance, heightDiff);
//            setFlyWheelSpeed(flywheelVelocity, gamepad1.a);
            if (gamepad1.y){
                turret.turret.setPosition(0.75);
            }
            if (gamepad1.b){
                turret.turnToRobotAngle(Math.toRadians(90));
            }
            if (gamepad1.a){
                turret.turnToRobotAngle(Math.toRadians(-90));
            }
            if (gamepad1.x){
                turret.turnToRobotAngle(0);
            }
            if (gamepad1.dpadLeftWasPressed()){
                turretPos += 0.01;
                turret.turret.setPosition(turretPos);
            }
            if (gamepad1.dpadRightWasPressed()){
                turretPos -= 0.01;
                turret.turret.setPosition(turretPos);
            }

            if (gamepad1.right_bumper){
//                turret.turret.setPower(1); //ccw
//                turret.turret2.setPower(1);
//                sleep(500);
//                for(double p = 0; p <= 1; p += 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                for(double p = 1; p >= 0; p -= 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                turret.turret.setPower(0);
//                turret.turret2.setPower(0);
            }
            if (gamepad1.right_trigger > 0.1){
//                turret.turret2.setPower(1); //ccw
//                turret.turret2.setPower(1);
//                sleep(500);
//                for(double p = 0; p <= 1; p += 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                for(double p = 1; p >= 0; p -= 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                turret.turret.setPower(0);
//                turret.turret2.setPower(0);
            }
            if (gamepad1.left_trigger > 0.1){
//                turret.turret2.setPower(-1); //cw
//                turret.turret2.setPower(1);
//                sleep(500);
//                for(double p = 0; p <= 1; p += 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                for(double p = 1; p >= 0; p -= 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                turret.turret.setPower(0);
//                turret.turret2.setPower(0);
            }
//            if (gamepad1.left_bumper){
//                turret.turnToRobotAngle(0);
//                sleep(5000);
//                for(double a = 0; a <= Math.PI; a += 0.01*Math.PI) {
//                    turret.turnToRobotAngle(a);
//                    sleep(100);
//                }
//                for(double a = 1; a >= -Math.PI; a -= 0.01*Math.PI) {
//                    turret.turnToRobotAngle(a);
//                    sleep(100);
//                }
//            }
//            if (gamepad1.dpad_up){
//                turretPos += 0.001;
//                turret.turret.setPosition(turretPos);
//            }
            if (gamepad1.dpad_down){
//                turret.turret.setPower(-1);  //cw
//                turret.turret2.setPower(-1);
                sleep(500);
//                for(double p = 0; p <= 1; p += 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                for(double p = 1; p >= 0; p -= 0.01) {
//                    turret.turret.setPosition(p);
//                    sleep(100);
//                }
//                turret.turret.setPower(0);
//                turret.turret2.setPower(0);
            }

            // Telemetry lines
            telemetry.addData("turret pos", turretPos);
//            telemetry.addData("turret current rotation", turret.turret.getCurrentAngle());
//            telemetry.addData("turret total rotation", turret.turret.getTotalRotation());
            telemetry.addData("0", constrain(AngleUnit.normalizeDegrees(0- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset - Robot.Constants.turretAngleOffset) /Robot.Constants.turretGearRatio);
            telemetry.addData("90",constrain(AngleUnit.normalizeDegrees(90- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset - Robot.Constants.turretAngleOffset) /Robot.Constants.turretGearRatio);
            telemetry.addData("45",constrain(AngleUnit.normalizeDegrees(45- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset - Robot.Constants.turretAngleOffset) /Robot.Constants.turretGearRatio);
            telemetry.addData("-90",constrain(AngleUnit.normalizeDegrees(-90- Robot.Constants.turretTargetRangeOffset) + Robot.Constants.turretTargetRangeOffset - Robot.Constants.turretAngleOffset) /Robot.Constants.turretGearRatio);
            //        turret.setTargetRotation(constrain(Range.scale(angle,Robot.Constants.turretLowAngle,Robot.Constants.turretHighAngle,Robot.Constants.turretScale0,Robot.Constants.turretScale1)));
            telemetry.update();

        }
    }
    public double constrain(double ogPos){
        return Range.clip(ogPos,Robot.Constants.turretClip0,Robot.Constants.turretClip1);
    }

};