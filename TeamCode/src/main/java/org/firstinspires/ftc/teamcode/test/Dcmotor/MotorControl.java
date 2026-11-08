package org.firstinspires.ftc.teamcode.test.Dcmotor;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic Motor Control", group="Linear Opmode")
public class MotorControl extends LinearOpMode {

    // Define the motor variable
    private DcMotor myMotor;

    @Override
    public void runOpMode() {
        // Initialize the motor object (must match your configuration name)
        myMotor = hardwareMap.get(DcMotor.class, "myMotor");

        // Set the direction (Reverse if necessary, depending on your build)
        // myMotor.setDirection(DcMotor.Direction.REVERSE);

        // BRAKE mode actively resists rotation when power is zero
        myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Read the left joystick's Y-axis (-1 to 1)
            // Push forward for negative, backward for positive
//            double power = -gamepad1.left_stick_y;
            double power = 0.8;

            // Send the power to the motor
            myMotor.setPower(power);

            // Send telemetry data to the Driver Station for debugging
            telemetry.addData("Motor Power", myMotor.getPower());
            telemetry.update();
        }
    }
}
