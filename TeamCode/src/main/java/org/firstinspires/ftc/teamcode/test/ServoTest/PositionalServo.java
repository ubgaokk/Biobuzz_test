package org.firstinspires.ftc.teamcode.test.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "REV Servo Position Example", group = "Concept")
public class PositionalServo extends LinearOpMode {

    // Declare the servo object
    private Servo revServo;

    @Override
    public void runOpMode() {

        /* Hardware Mapping:
         * The string "turret" must match the name configured
         * on the Driver Station / Control Hub hardware configuration.
         */
        revServo = hardwareMap.get(Servo.class, "turret");

        // Optional: Initialize the servo to its center position before the match starts
        revServo.setPosition(0.5);

        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Run until the driver presses STOP
        while (opModeIsActive()) {

            // Check gamepad buttons to command specific positions
            if (gamepad1.y) {
                // Move to position 0.0 (Far Left / -135 degrees)
                revServo.setPosition(0.33);
            }
            else if (gamepad1.x || gamepad1.b) {
                // Move to position 0.5 (Center / 0 degrees)
                revServo.setPosition(0.5);
            }
            else if (gamepad1.a) {
                // Move to position 1.0 (Far Right / +135 degrees)
                revServo.setPosition(0.66);
            }

            // Send telemetry data back to the Driver Station to monitor current status
            telemetry.addData("Target Position Set", revServo.getPosition());
            telemetry.update();
        }
    }
}
