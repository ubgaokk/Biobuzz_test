package org.firstinspires.ftc.teamcode.test.ServoTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;


@TeleOp(name = "CR goBILDA Speed Servo Test", group = "Test")
public class GobildaServo_CR_ extends LinearOpMode {

    // 1.0 is the maximum CR servo power (full speed). Spin direction is set with setDirection() below.
    private static final double SPIN_POWER = 1.0;

    private CRServo speedServo;

    @Override
    public void runOpMode() {

        // "speedServo" must match the name in Robot Configuration (as a Continuous Rotation Servo)
        speedServo = hardwareMap.get(CRServo.class, "speedServo");
        // Reverse the spin direction. Use FORWARD to switch back.
        speedServo.setDirection(DcMotorSimple.Direction.FORWARD);
        // goBILDA servos reach full speed at 500-2500 us; the SDK default is only 600-2400 us.
        if (speedServo instanceof CRServoImplEx) {
            ((CRServoImplEx) speedServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        speedServo.setPower(0);

        telemetry.addLine("goBILDA CR Servo Test Ready");
        telemetry.addLine("Spins continuously in one direction at full speed");
        telemetry.addLine("B = stop, A = resume");
        telemetry.update();

        waitForStart();

        boolean spinning = true;

        while (opModeIsActive()) {

            if (gamepad1.b) {
                spinning = false;
            } else if (gamepad1.a) {
                spinning = true;
            }

            double power = spinning ? SPIN_POWER : 0.0;
            speedServo.setPower(power);

            telemetry.addData("Spinning", spinning);
            telemetry.addData("Servo Power", "%.2f", power);
            telemetry.update();
        }

        // Safety: stop servo when OpMode ends
        speedServo.setPower(0);
    }
}
