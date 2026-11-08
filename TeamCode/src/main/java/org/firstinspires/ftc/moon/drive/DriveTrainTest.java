package org.firstinspires.ftc.moon.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drivetrain Test", group = "Test")
public class DriveTrainTest extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "W_FR");
        BL = hardwareMap.get(DcMotor.class, "W_BL");
        BR = hardwareMap.get(DcMotor.class, "W_BR");
        FL = hardwareMap.get(DcMotor.class, "W_FL");

        // Make sure no RUN_TO_POSITION mode left over from another OpMode blocks setPower().
        for (DcMotor m : new DcMotor[]{FL, FR, BL, BR}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Common drivetrain direction setup.
        // If the robot drives backward when pushing forward, swap these.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Drivetrain Test Ready");
        telemetry.addLine("Left stick: move");
        telemetry.addLine("Right stick X: turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;    // strafe
            double rx = gamepad1.right_stick_x;  // turn

            // Mecanum drive formula
            double flPower = y + x + rx;
            double blPower = y - x + rx;
            double frPower = y - x - rx;
            double brPower = y + x - rx;

            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            if (max > 1.0) {
                flPower /= max;
                blPower /= max;
                frPower /= max;
                brPower /= max;
            }

            FL.setPower(flPower);
            BL.setPower(blPower);
            FR.setPower(frPower);
            BR.setPower(brPower);

            telemetry.addData("Sticks (y, x, rx)", "%.2f, %.2f, %.2f", y, x, rx);
            telemetry.addData("FL power", flPower);
            telemetry.addData("FR power", frPower);
            telemetry.addData("BL power", blPower);
            telemetry.addData("BR power", brPower);
            telemetry.update();
        }
    }
}