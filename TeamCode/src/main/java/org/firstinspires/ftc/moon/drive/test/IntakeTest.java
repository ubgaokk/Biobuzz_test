package org.firstinspires.ftc.moon.drive.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;

    private static final double SERVO_POWER = 1.0;

    private CRServo speedServo;
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "W_FR");
        BL = hardwareMap.get(DcMotor.class, "W_BL");
        BR = hardwareMap.get(DcMotor.class, "W_BR");
        FL = hardwareMap.get(DcMotor.class, "W_FL");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Make sure no RUN_TO_POSITION mode left over from another OpMode blocks setPower().
        for (DcMotor m : new DcMotor[]{FL, FR, BL, BR}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // BRAKE mode actively resists rotation when power is zero
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        // Intake configuration
        // "speedServo" must match the name in Robot Configuration, configured as "Continuous Rotation Servo"
        speedServo = hardwareMap.get(CRServo.class, "speedServo");
        // Reverse the spin direction. Use FORWARD to switch back.
        speedServo.setDirection(DcMotorSimple.Direction.FORWARD);
        // goBILDA servos reach full speed at 500-2500 us; the SDK default is only 600-2400 us.
        if (speedServo instanceof CRServoImplEx) {
            ((CRServoImplEx) speedServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        speedServo.setPower(0);

        telemetry.addLine("Drivetrain + Intake Test Ready");
        telemetry.addLine("Left stick: move, Right stick X: turn");
        telemetry.addLine("Servo spins continuously: A = forward, X = reverse, B = stop");
        telemetry.update();

        waitForStart();
        double servoPower = SERVO_POWER;
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


//            if (gamepad1.a) {
//                servoPower = SERVO_POWER;
//            } else if (gamepad1.x) {
//                servoPower = -SERVO_POWER;
//            } else if (gamepad1.b) {
//                servoPower = 0.0;
//            }
//            servoPower = SERVO_POWER;
            speedServo.setPower(servoPower);

            double intakePower = -0.9;

            // Send the power to the motor
            intakeMotor.setPower(intakePower);

            telemetry.addData("Servo power", servoPower);
            telemetry.addData("Sticks (y, x, rx)", "%.2f, %.2f, %.2f", y, x, rx);
            telemetry.addData("FL power", flPower);
            telemetry.addData("FR power", frPower);
            telemetry.addData("BL power", blPower);
            telemetry.addData("BR power", brPower);
            telemetry.update();
        }
    }
}