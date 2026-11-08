package org.firstinspires.ftc.teamcode.ApriltagDet;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

@TeleOp(name = "Limelight AprilTag", group = "Sensor")
public class AprilTagDetection extends OpMode {

    private static final double REFERENCE_X = -58.3727; //inches
    private static final double REFERENCE_Y = 55.6425;
    private static final double REFERENCE_Z = 29.5;
    private static final double X_ANGLE_TARGET_DEG = 0.0;
    private static final double X_ANGLE_DEADBAND_DEG = 1.0;
    private static final double AUTO_AIM_KP = 0.03;
    private static final double MAX_SERVO_POWER = 0.1;

    private Limelight3A limelight;
    private IMU imu;
    private CRServo axonCR;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        axonCR = hardwareMap.get(CRServo.class, "servorot");

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
        double servoPower = 0.0;
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
                    double xError = -1*(xAngle - X_ANGLE_TARGET_DEG);
                    if (Math.abs(xError) > X_ANGLE_DEADBAND_DEG) {
                        servoPower = Range.clip(-xError * AUTO_AIM_KP, -MAX_SERVO_POWER, MAX_SERVO_POWER);
                    }
                }
//                double relativeX = fiducial.getRobotPoseTargetSpace().getPosition().x;
//                double relativeY = fiducial.getRobotPoseTargetSpace().getPosition().y;
//                double relativeZ = fiducial.getRobotPoseTargetSpace().getPosition().z;

                // Get 3D distance in meters (Y is generally used as depth/distance)
                double distance = fiducial.getRobotPoseTargetSpace().getPosition().z;

                telemetry.addData("Tag ID", id);
                telemetry.addData("X Angle (deg)", xAngle);
                telemetry.addData("Distance (m)", distance);
            }
        } else {
            telemetry.addData("Limelight", "No AprilTags detected");
        }
        axonCR.setPower(servoPower);
        telemetry.addData("Yaw (deg)", yawDegrees);
        telemetry.addData("Tracked X Angle (deg)", hasTarget ? trackedXAngle : "N/A");
        telemetry.addData("Servo Power", servoPower);
        telemetry.update();
    }
}