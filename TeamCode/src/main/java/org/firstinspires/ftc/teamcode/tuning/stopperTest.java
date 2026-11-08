package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
@Disabled

@TeleOp(name = "Stopper Test")
// Program used to test hood positions
public class stopperTest extends LinearOpMode {
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Telemetry telemetry = dashboard.getTelemetry();
    Stopper hood;
 // 0.27 close 0.04 open
    double HoodAngle=0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) { // Lowest angle
                HoodAngle = 0;
            }
            if (gamepad1.yWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 1;
            }
            if (gamepad1.dpadUpWasPressed()) { // Increase hood position
                HoodAngle += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()) { // Decrease hood position
                HoodAngle -= 0.01;
            }
//            hood.turnToAngle(HoodAngle);
            hood.STOPPER.setPosition(HoodAngle);
            // Telemetry lines
            telemetry.addData("Hood target angle", HoodAngle);
//            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.addData("Hood pos", hood.STOPPER.getPosition());
            telemetry.addData("outside range", hood.STOPPER.commandedOutsideRange());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Stopper(hardwareMap);
        HoodAngle = 0;
    }
}
