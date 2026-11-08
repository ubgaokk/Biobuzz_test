package org.firstinspires.ftc.teamcode.test.ServoTest;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name="Axon MAX MK2 Example", group="Examples")
public class ServoOpMode extends LinearOpMode {

    // If using Continuous Rotation Mode
    private CRServo axonCR;

    // If using Servo Mode
    private Servo axonServo;

    @Override
    public void runOpMode() {
        // Map from Robot Configuration (Driver Station)
        axonCR = hardwareMap.get(CRServo.class, "Lservo");
        // axonServo = hardwareMap.get(Servo.class, "axonServo");

        waitForStart();

        while (opModeIsActive()) {
            // Example: CR mode control with gamepad
//            double power = -gamepad1.left_stick_y; // joystick up/down
            double power = 0.8;
            axonCR.setPower(power);

            // Example: Servo mode control (uncomment if in Servo Mode)
            // double pos = (gamepad1.right_stick_x + 1) / 2.0; // map -1..1 → 0..1
            // axonServo.setPosition(pos);

            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}