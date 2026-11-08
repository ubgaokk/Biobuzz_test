package org.firstinspires.ftc.teamcode.ObjectDetection;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;

import java.util.Locale;

/**
 * Simple color blob detector for FTC VisionPortal (2025 SDK).
 * Splits the frame into left and right ROIs, detects average color,
 * and determines which side contains the target.
 */
public class ColorBlobProcessor implements VisionProcessor {

    // Detection result (thread-safe with volatile)
    private volatile String detectionResult = "Not Found";

    // Define HSV color boundaries for red (two ranges due to hue wrap)
    private static final Scalar LOWER_RED1 = new Scalar(0, 120, 70);
    private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED2 = new Scalar(170, 120, 70);
    private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);

    // Define regions of interest (adjust sizes to match your camera stream)
    private final Rect ROI2 = new Rect(0, 0, 160, 240);   // Left half
    private final Rect ROI1 = new Rect(361, 0, 160, 240); // Right half

    // Mats for processing
    private final Mat hsv = new Mat();
    private final Mat mask1 = new Mat();
    private final Mat mask2 = new Mat();
    private final Mat redMask = new Mat();

    // Real-world height of the red brick in mm (adjust as needed)
    private static final double BRICK_REAL_HEIGHT_MM = 58.0; // Example: 32mm for a 2x4 LEGO brick
    // Camera focal length in pixels (should be calibrated for your camera)
    private static final double CAMERA_FOCAL_LENGTH_PX = 644.21; // zed2 529.6;//logitech270 822.317; // Example value, adjust after calibration

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Called once at VisionPortal initialization.
        // No calibration use in this example, but available if needed.
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        // Convert input frame to HSV color space
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold to isolate red (two ranges)
        Core.inRange(hsv, LOWER_RED1, UPPER_RED1, mask1);
        Core.inRange(hsv, LOWER_RED2, UPPER_RED2, mask2);
        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, redMask);

        // Morphological operations to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);

        // Find contours in the mask
        java.util.List<org.opencv.core.MatOfPoint> contours = new java.util.ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        Rect boundingRect = null;
        for (org.opencv.core.MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                boundingRect = Imgproc.boundingRect(contour);
            }
        }

        if (boundingRect != null && maxArea > 500) { // 500: area threshold, adjust as needed
            Imgproc.rectangle(frame, boundingRect, new Scalar(0, 255, 0), 5);
            // Depth estimation
            double pixelHeight = boundingRect.height;
            double depthMM = (BRICK_REAL_HEIGHT_MM * CAMERA_FOCAL_LENGTH_PX) / pixelHeight;
            String depthText = String.format(Locale.US, "Depth: %.1f mm", depthMM);
            Imgproc.putText(frame, depthText, new Point(boundingRect.x, boundingRect.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,255,0), 2);
            detectionResult = "Red Brick Detected, " + depthText;
        } else {
            detectionResult = "Not Found";
        }

        // Optionally, show mask for debugging (comment out if not needed)
        // Imgproc.putText(frame, "Red Mask", new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0,255,0), 2);

        // Return the annotated frame (displayed on DS if enabled)
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Optional: draw overlays directly on DS canvas.
        // We already draw rectangles on the Mat, so this can stay empty.
    }

    /** Public getter for the detection result. */
    public String getDetectionResult() {
        return detectionResult;
    }
}