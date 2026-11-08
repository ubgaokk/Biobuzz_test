package org.firstinspires.ftc.teamcode.test.ServoTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Yellow Ball Detection Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // Same physical REV Color Sensor V3
        colorSensor = hardwareMap.get(
                NormalizedColorSensor.class,
                "colorSensor"
        );

        distanceSensor = hardwareMap.get(
                DistanceSensor.class,
                "colorSensor"
        );

        // Optional: adjust gain if readings are too weak/strong
        colorSensor.setGain(20.0f);

        telemetry.addLine("REV Color Sensor V3 Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;
            float alpha = colors.alpha;

            double distanceMM =
                    distanceSensor.getDistance(DistanceUnit.MM);

            boolean ballPresent = distanceMM < 75;

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

            telemetry.addData("Red", "%.3f", red);
            telemetry.addData("Green", "%.3f", green);
            telemetry.addData("Blue", "%.3f", blue);
            telemetry.addData("Alpha", "%.3f", alpha);

            telemetry.addData(
                    "Distance (mm)",
                    "%.1f",
                    distanceMM
            );

            if (!ballPresent) {
                telemetry.addLine("NO BALL");
            } else {
                telemetry.addLine(">>> " + detectedColor + " BALL <<<");
            }

            telemetry.update();

            sleep(20);
        }
    }
}