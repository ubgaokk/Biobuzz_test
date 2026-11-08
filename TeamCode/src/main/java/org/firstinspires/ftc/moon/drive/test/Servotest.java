package org.firstinspires.ftc.moon.drive.test;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "GoBILDA Speed Servo Test", group = "Test")
public class Servotest extends LinearOpMode {

    private CRServo speedServo;

    @Override
    public void runOpMode() {

        speedServo = hardwareMap.get(CRServo.class, "Lservo");

        telemetry.addLine("Ready to test goBILDA speed servo");
        telemetry.addLine("Press START to spin clockwise");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Positive power usually spins one direction.
            // If this spins counterclockwise, change 1.0 to -1.0.
            speedServo.setPower(1.0);

            telemetry.addData("Servo Power", 1.0);
            telemetry.addLine("Spinning clockwise");
            telemetry.update();
        }

        speedServo.setPower(0);
    }
}