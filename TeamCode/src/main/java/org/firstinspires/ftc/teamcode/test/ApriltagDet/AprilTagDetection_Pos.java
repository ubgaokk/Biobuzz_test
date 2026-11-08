package org.firstinspires.ftc.teamcode.ApriltagDet;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

@TeleOp(name = "Limelight AprilTag position", group = "Sensor")
public class AprilTagDetection_Pos extends OpMode {

    private static final double REFERENCE_X = -58.3727; //inches
    private static final double REFERENCE_Y = 55.6425;
    private static final double REFERENCE_Z = 29.5;
    private static final double X_ANGLE_TARGET_DEG = 0.0;
    private static final double X_ANGLE_DEADBAND_DEG = 2;
    private static final double AUTO_AIM_KP = 0.05;
    private static final double MAX_POSITION_STEP_PER_LOOP = 20;
    private static final double MIN_SERVO_POSITION = 0.0;
    private static final double MAX_SERVO_POSITION = 1.0;
    private static final double SERVO_CENTER_POSITION = 0.5;

    private Limelight3A limelight;
    private IMU imu;
    private Servo positionalServo;
    private double servoPositionCmd = SERVO_CENTER_POSITION;

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

        telemetry.addData("Reference Point", "X=%.4f, Y=%.4f, Z=%.4f", REFERENCE_X, REFERENCE_Y, REFERENCE_Z);
        telemetry.update();
    }

    @Override
    public void loop() {


        double yawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double desiredPositionStep = 0.0;
        boolean hasTarget = false;
        double trackedXAngle = 0.0;
        limelight.updateRobotOrientation(yawDegrees);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                double xAngle = fiducial.getTargetXDegrees(); // Left/Right
                if (!hasTarget) {
                    hasTarget = true;
                    trackedXAngle = xAngle;
                    double xError = xAngle - X_ANGLE_TARGET_DEG;
                    if (Math.abs(xError) > X_ANGLE_DEADBAND_DEG) {
                        desiredPositionStep = Range.clip(
                                xError * AUTO_AIM_KP,
                                -MAX_POSITION_STEP_PER_LOOP,
                                MAX_POSITION_STEP_PER_LOOP
                        );
                        desiredPositionStep = desiredPositionStep/135.0 *0.5; // Normalized
                    }
                }
//                double relativeX = fiducial.getRobotPoseTargetSpace().getPosition().x;
//                double relativeY = fiducial.getRobotPoseTargetSpace().getPosition().y;
//                double relativeZ = fiducial.getRobotPoseTargetSpace().getPosition().z;

                // Get 3D distance in meters (Y is generally used as depth/distance)
                double distance = fiducial.getRobotPoseTargetSpace().getPosition().z;

                telemetry.addData("Tag ID", id);
//                telemetry.addData("X Angle (deg)", xAngle);
                telemetry.addData("Distance (m)", distance);
            }
        } else {
            telemetry.addData("Limelight", "No AprilTags detected");
        }
        servoPositionCmd = Range.clip(
                servoPositionCmd + desiredPositionStep,
                MIN_SERVO_POSITION,
                MAX_SERVO_POSITION
        );
        positionalServo.setPosition(servoPositionCmd);
        telemetry.addData("desiredPositionStep", desiredPositionStep);
        telemetry.addData("Yaw (deg)", yawDegrees);
        telemetry.addData("Tracked X Angle (deg)", hasTarget ? trackedXAngle : "N/A");
        telemetry.addData("Servo Position", servoPositionCmd);
        telemetry.update();
    }
}
