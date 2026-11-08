package org.firstinspires.ftc.teamcode.ObjectDetection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "AutoOpMode with Object Detection", group = "Robot")
public class MyAutoOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private ColorBlobProcessor colorBlobProcessor;

    @Override
    public void runOpMode() {
        // Create the VisionProcessor
        colorBlobProcessor = new ColorBlobProcessor();

        // Create the VisionPortal using the Builder pattern
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorBlobProcessor)
                .build();

        telemetry.addData("Status", "Vision initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get data from your processor
            String detectionResult = colorBlobProcessor.getDetectionResult();

            // Based on the detection, drive the robot
            // Example:
            // if (detectionResult.equals("Left")) {
            //    // Drive left
            // } else if (detectionResult.equals("Right")) {
            //    // Drive right
            // }

            telemetry.addData("Detection", detectionResult);
            telemetry.update();

            sleep(50); // Pause for a short period
        }

        // Stop streaming at the end of the OpMode
        visionPortal.close();
    }
}