package org.firstinspires.ftc.teamcode.ApriltagDet;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Limelight AprilTag position", group = "Sensor")
public class AprilTagDetection_Pos_old extends OpMode {

    private static final double REFERENCE_X = -58.3727; //inches
    private static final double REFERENCE_Y = 55.6425;
    private static final double REFERENCE_Z = 29.5;
    private static final double X_ANGLE_TARGET_DEG = 0.0;
    private static final double X_ANGLE_DEADBAND_DEG = 2;

    // PID gains: error is in degrees, output is servo travel degrees per second.
    private static final double AUTO_AIM_KP = 2.5;
    private static final double AUTO_AIM_KI = 0.8;
    private static final double AUTO_AIM_KD = 0.12;

    // Output/robustness limits.
    private static final double MAX_RATE_DEG_PER_SEC = 300.0;
    private static final double MAX_INTEGRAL_DEG_PER_SEC = 60.0;
    private static final double SERVO_TRAVEL_DEG = 270.0;
    private static final double MIN_LOOP_SECONDS = 0.001;
    private static final double MAX_LOOP_SECONDS = 0.25;
    // Time constant of the low-pass filter applied to the derivative term.
    private static final double D_FILTER_TAU_SECONDS = 0.05;
    // How long a target may be missing before the integrator is released.
    private static final double TARGET_LOST_TIMEOUT_SECONDS = 0.5;
    private static final double MAX_TRACKABLE_X_ANGLE_DEG = 45.0;

    private static final double MIN_SERVO_POSITION = 0.0;
    private static final double MAX_SERVO_POSITION = 1.0;
    private static final double SERVO_CENTER_POSITION = 0.5;

    private Limelight3A limelight;
    private IMU imu;
    private Servo positionalServo;
    private double servoPositionCmd = SERVO_CENTER_POSITION;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private double integralTerm = 0.0;
    private double filteredDerivative = 0.0;
    private double lastError = 0.0;
    private boolean hasLastError = false;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        positionalServo = hardwareMap.get(Servo.class, "servo_pos");
        positionalServo.setPosition(servoPositionCmd);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with mounting parameters
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Optional: Reset yaw to 0 at the start of the match
        imu.resetYaw();

        limelight.start();

        resetControlState();

        telemetry.addData("Reference Point", "X=%.4f, Y=%.4f, Z=%.4f", REFERENCE_X, REFERENCE_Y, REFERENCE_Z);
        telemetry.update();
    }

    @Override
    public void start() {
        resetControlState();
    }

    private void resetControlState() {
        integralTerm = 0.0;
        filteredDerivative = 0.0;
        lastError = 0.0;
        hasLastError = false;
        loopTimer.reset();
        targetLostTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Range.clip(loopTimer.seconds(), MIN_LOOP_SECONDS, MAX_LOOP_SECONDS);
        loopTimer.reset();

        double yawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yawDegrees);

        LLResultTypes.FiducialResult trackedFiducial = null;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    double xAngle = fiducial.getTargetXDegrees();
                    // Reject bogus or far off-axis detections before they can drive the servo.
                    if (Double.isNaN(xAngle) || Math.abs(xAngle) > MAX_TRACKABLE_X_ANGLE_DEG) {
                        continue;
                    }
                    if (trackedFiducial == null
                            || Math.abs(xAngle - X_ANGLE_TARGET_DEG)
                            < Math.abs(trackedFiducial.getTargetXDegrees() - X_ANGLE_TARGET_DEG)) {
                        trackedFiducial = fiducial;
                    }
                    telemetry.addData("Tag ID", fiducial.getFiducialId());
                    telemetry.addData("Distance (m)", getTagDistanceMeters(fiducial));
                }
            }
        }

        boolean hasTarget = trackedFiducial != null;
        double trackedXAngle = hasTarget ? trackedFiducial.getTargetXDegrees() : 0.0;
        double xError = 0.0;
        double pTerm = 0.0;
        double dTerm = 0.0;
        double rateDegPerSec = 0.0;

        if (hasTarget) {
            targetLostTimer.reset();
            xError = trackedXAngle - X_ANGLE_TARGET_DEG;

            if (Math.abs(xError) <= X_ANGLE_DEADBAND_DEG) {
                // Inside the deadband: hold still and bleed off the integrator to avoid creep.
                integralTerm = 0.0;
                filteredDerivative = 0.0;
                lastError = xError;
                hasLastError = true;
            } else {
                pTerm = AUTO_AIM_KP * xError;

                double rawDerivative = hasLastError ? (xError - lastError) / dt : 0.0;
                double alpha = dt / (D_FILTER_TAU_SECONDS + dt);
                filteredDerivative += alpha * (rawDerivative - filteredDerivative);
                dTerm = AUTO_AIM_KD * filteredDerivative;

                double candidateIntegral = Range.clip(
                        integralTerm + AUTO_AIM_KI * xError * dt,
                        -MAX_INTEGRAL_DEG_PER_SEC,
                        MAX_INTEGRAL_DEG_PER_SEC
                );
                double unsaturated = pTerm + candidateIntegral + dTerm;
                double saturated = Range.clip(unsaturated, -MAX_RATE_DEG_PER_SEC, MAX_RATE_DEG_PER_SEC);
                // Conditional integration: only accumulate when it does not push further into saturation.
                if (unsaturated == saturated || Math.signum(xError) != Math.signum(unsaturated)) {
                    integralTerm = candidateIntegral;
                }

                rateDegPerSec = Range.clip(
                        pTerm + integralTerm + dTerm,
                        -MAX_RATE_DEG_PER_SEC,
                        MAX_RATE_DEG_PER_SEC
                );

                lastError = xError;
                hasLastError = true;
            }
        } else {
            telemetry.addData("Limelight", "No AprilTags detected");
            filteredDerivative = 0.0;
            hasLastError = false;
            if (targetLostTimer.seconds() > TARGET_LOST_TIMEOUT_SECONDS) {
                integralTerm = 0.0;
            }
        }

        double desiredPositionStep = (rateDegPerSec * dt) / SERVO_TRAVEL_DEG;
        double requestedPosition = servoPositionCmd + desiredPositionStep;
        double nextPosition = Range.clip(requestedPosition, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        // Travel limit reached: stop winding up the integrator against the end stop.
        if (Math.abs(requestedPosition - nextPosition) > 1e-9) {
            integralTerm = 0.0;
        }
        servoPositionCmd = nextPosition;
        positionalServo.setPosition(servoPositionCmd);
        telemetry.addData("dt (s)", "%.4f", dt);
        telemetry.addData("X Error (deg)", hasTarget ? String.format("%.2f", xError) : "N/A");
        telemetry.addData("PID (P/I/D)", "%.2f / %.2f / %.2f", pTerm, integralTerm, dTerm);
        telemetry.addData("Rate (deg/s)", "%.2f", rateDegPerSec);
        telemetry.addData("desiredPositionStep", desiredPositionStep);
        telemetry.addData("Yaw (deg)", yawDegrees);
        telemetry.addData("Tracked X Angle (deg)", hasTarget ? trackedXAngle : "N/A");
        telemetry.addData("Servo Position", servoPositionCmd);
        telemetry.update();
    }

    private double getTagDistanceMeters(LLResultTypes.FiducialResult fiducial) {
        // Pose data is not always populated; guard so a null pose cannot kill the loop.
        try {
            if (fiducial.getRobotPoseTargetSpace() == null
                    || fiducial.getRobotPoseTargetSpace().getPosition() == null) {
                return Double.NaN;
            }
            return fiducial.getRobotPoseTargetSpace().getPosition().z;
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }
}
