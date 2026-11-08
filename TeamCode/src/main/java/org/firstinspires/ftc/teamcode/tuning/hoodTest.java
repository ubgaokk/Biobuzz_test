package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Hood Test")
// Program used to test hood positions
public class hoodTest extends LinearOpMode {
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    Hood hood;

    double HoodAngle=50;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.aWasPressed()) { // Lowest angle
//                HoodAngle = 0;
//            }
            if (gamepad1.yWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 35;
                hood.turnToAngle(35*Math.PI/180);
            }
            if (gamepad1.xWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 45;
                hood.turnToAngle(45*Math.PI/180);
            }
            if (gamepad1.aWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 29;
                hood.turnToAngle(29*Math.PI/180);
            }
            if (gamepad1.dpadUpWasPressed()) { // Increase hood position
                HoodAngle += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()) { // Decrease hood position
                HoodAngle -= 0.01;
            }
//            hood.turnToAngle(HoodAngle);
//            hood.setPosition(HoodAngle);
            hood.HOOD.setPosition(HoodAngle);
            // Telemetry lines
            telemetry.addData("Hood supposed pos", Range.scale(HoodAngle, 0, 1, Robot.Constants.hoodScale0, Robot.Constants.hoodScale1));
            telemetry.addData("Hood target angle", HoodAngle);
            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.addData("Hood pos", hood.HOOD.getPosition());
            telemetry.addData("outside range", hood.HOOD.commandedOutsideRange());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Hood(hardwareMap);
        HoodAngle = 0;
    }
}
