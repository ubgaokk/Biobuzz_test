package org.firstinspires.ftc.teamcode.test.ServoTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "goBILDA Servo Color Sort", group = "Test")
public class GobildaServo extends LinearOpMode {

    // Full travel of the servo in degrees.
    private static final double SERVO_RANGE_DEGREES = 180.0;

    private static final double POS_0_DEG  = 0.0 / SERVO_RANGE_DEGREES;
    private static final double POS_90_DEG = 90.0 / SERVO_RANGE_DEGREES;

    // A ball is considered present closer than this distance.
    private static final double BALL_DISTANCE_MM = 75.0;

    private Servo sortServo;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // Names must match the Robot Configuration
        sortServo = hardwareMap.get(Servo.class, "speedServo");

        // Same physical REV Color Sensor V3
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // Optional: adjust gain if readings are too weak/strong
        colorSensor.setGain(20.0f);

        telemetry.addLine("goBILDA Servo Color Sort Ready");
        telemetry.addLine("Yellow ball -> 0 deg, otherwise -> 90 deg");
        telemetry.update();

        waitForStart();

        sortServo.setPosition(POS_90_DEG);

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;
            float alpha = colors.alpha;

            double distanceMM = distanceSensor.getDistance(DistanceUnit.MM);

            boolean ballPresent = distanceMM < BALL_DISTANCE_MM;

            /*
             * Classify by whichever channel is the strongest:
             * green max -> YELLOW, red max -> RED, blue max -> BLUE.
             */
            String detectedColor;
            if (green >= red && green >= blue) {
                detectedColor = "YELLOW";
            } else if (red >= green && red >= blue) {
                detectedColor = "RED";
            } else {
                detectedColor = "BLUE";
            }

            boolean yellowBall = ballPresent && detectedColor.equals("YELLOW");

            double targetPosition = yellowBall ? POS_0_DEG : POS_90_DEG;
            sortServo.setPosition(targetPosition);

            telemetry.addData("Red", "%.3f", red);
            telemetry.addData("Green", "%.3f", green);
            telemetry.addData("Blue", "%.3f", blue);
            telemetry.addData("Alpha", "%.3f", alpha);
            telemetry.addData("Distance (mm)", "%.1f", distanceMM);

            if (!ballPresent) {
                telemetry.addLine("NO BALL");
            } else {
                telemetry.addLine(">>> " + detectedColor + " BALL <<<");
            }

            telemetry.addData("Servo Target", yellowBall ? "0 deg" : "90 deg");
            telemetry.addData("Servo Position", "%.3f", sortServo.getPosition());
            telemetry.update();

            sleep(20);
        }
    }
}
