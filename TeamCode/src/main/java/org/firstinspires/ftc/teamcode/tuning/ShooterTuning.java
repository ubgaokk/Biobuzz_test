package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Disabled

@TeleOp(name = "stuner")
@Config
// Press RB to make hood and flywheel move to right position/rpm, and press LB to intake. Limelight
// is currently not seeing?? idk how to operate haha
public class ShooterTuning extends OpMode {

    public static double HOOD_ANGLE_DEG = 0;     // degrees
    public static double FLYWHEEL_RPM = 0;     // rpm

    Shooter shooter;
    Limelight limelight;
    Intake intake;
//    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        limelight.update(telemetry);
        // Adjust flywheel and hood angles
        if (gamepad1.dpadUpWasPressed())
        {
            FLYWHEEL_RPM+=10;
        }
        else if (gamepad1.dpadDownWasPressed())
        {
            FLYWHEEL_RPM-=10;
        }
        else if (gamepad1.yWasPressed())
        {
            HOOD_ANGLE_DEG++;
        }
        else if (gamepad1.aWasPressed())
        {
            HOOD_ANGLE_DEG--;
        }

        // Convert to radians
        double hoodAngleRad = Math.toRadians(HOOD_ANGLE_DEG);

        if (gamepad1.right_bumper) {
            // Spin flywheel
            shooter.getFlywheel().spinTo(FLYWHEEL_RPM);

            // Move hood + turret to test angles
            shooter.getHood().turnToAngle(hoodAngleRad);
        }
        if (gamepad1.left_bumper) {
            intake.intake();
        }

        // Stop intake and flywheel if bumpers not pressed
        intake.stop();
        shooter.getFlywheel().setPower(0);

        // Telemetry
        telemetry.addData("target hood angle", HOOD_ANGLE_DEG);
        telemetry.addData("cur hood angle", Math.toDegrees(shooter.getHood().getAngle()));

        telemetry.addData("target flywheel speed", FLYWHEEL_RPM);
        telemetry.addData("cur flywheel speed", shooter.getFlywheel().getVel());

        telemetry.addData("lmlight distance to goal", limelight.getDistanceToGoal()); // should be inches
        // offsetted to center of robot

        telemetry.update();
    }
}