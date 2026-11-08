package org.firstinspires.ftc.teamcode.test.ServoTest;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Axon MAX MK2 Example", group="Examples")
public class ServoOpMode_test extends LinearOpMode {

    // If using Continuous Rotation Mode
    private CRServo axonCR;

    // If using Servo Mode
    private Servo axonServo;

    @Override
    public void runOpMode() {
        // Map from Robot Configuration (Driver Station)
        axonCR = hardwareMap.get(CRServo.class, "servorot");
        // axonServo = hardwareMap.get(Servo.class, "axonServo");

        waitForStart();

        while (opModeIsActive()) {
            // Example: CR mode control with gamepad
//            double power = -gamepad1.left_stick_y; // joystick up/down
//            axonCR.setPower(power);
            telemetry.addData("Power", 0.05);
            axonCR.setPower(0.05); // probably no motion
            telemetry.update();
            sleep(10000);

            telemetry.addData("Power", 0.1);
            axonCR.setPower(0.10); // maybe starts
            telemetry.update();
            sleep(2000);

            axonCR.setPower(0.15); // more likely
            telemetry.addData("Power", 0.15);
            telemetry.update();
            sleep(2000);

            axonCR.setPower(0);

            // Example: Servo mode control (uncomment if in Servo Mode)
            // double pos = (gamepad1.right_stick_x + 1) / 2.0; // map -1..1 → 0..1
            // axonServo.setPosition(pos);

//            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}