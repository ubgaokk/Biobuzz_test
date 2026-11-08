package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;


public class Limelight {
    Limelight3A limelight;
    double heading;

    public Limelight(HardwareMap hardwareMap) // initialization constructor
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // currently tuned pipeline
        limelight.setPollRateHz(100); // update data 100 times/s
        limelight.start(); // starts limelight
    }
    public void switchToAprilTag(){
        limelight.pipelineSwitch(0); // currently tuned pipeline
        limelight.start();
    }
    public void switchToBalls(){
        limelight.pipelineSwitch(1); // currently tuned pipeline
    }

    public void updateHeading(double headingDegrees){
        heading = headingDegrees;
        limelight.updateRobotOrientation(headingDegrees);
    }

//    public List getArtifacts()
//    {
//        LLResult result = limelight.getLatestResult();
//    }

    public boolean findLoadingArtifacts(double xpos) // checks for artifacts in loading zone
    {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            RobotLog.a("Limelight No data available");
            return false;
        }

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        List<Double> xPositions = new java.util.ArrayList<>();
//        java.util.Collections.sort(xPositions);

        int count = 0;

        for (LLResultTypes.DetectorResult dr : detectorResults) {
            double x = dr.getTargetXPixels();   // x position of this detection
            // x increases towards loading zone
            if (x > xpos) {
                count++;
            }
        }

        if (count > 0)
        {
            return true;
        }
        else return false;
    }

    public LLResult getResults()
    {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            RobotLog.a("Limelight No data available");
            return null;
        }
        else return result;
    }

    public double updateMostBallsX(boolean active, Telemetry telemetry){
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("Limelight No data available");
            }
            return Double.NaN;
        }

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

        if (detectorResults == null || detectorResults.size() == 0) {
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("Balls None detected");
            }
            return Double.NaN;
        }

        // 1. Collect X pixel positions
        List<Double> xPositions = new java.util.ArrayList<>();

        for (LLResultTypes.DetectorResult dr : detectorResults) {
            if (dr.getClassName().equalsIgnoreCase("p") || dr.getClassName().equalsIgnoreCase("g")) {
                xPositions.add(dr.getTargetXPixels());
            }
        }

        if (xPositions.size() == 0) {
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("Balls No ball class detected");
            }
            return Double.NaN;
        }

        // 2. Sort X positions
        java.util.Collections.sort(xPositions);
        java.util.Collections.reverse(xPositions);

        // 3. Sliding window
        final double WINDOW_WIDTH = 18*23.40; // pixels (tune this)
        int left = 0;
        int bestCount = 0;
        double bestCenter = Double.NaN;
        double bestWindow = Double.NaN;

        List<Double> yDistances = new java.util.ArrayList<>(); // robot distances to balls
        // drop + replace distances w/ optimized artifacts

        for (int right = 0; right < xPositions.size(); right++) {

            while (Math.abs(xPositions.get(right) - xPositions.get(left)) > WINDOW_WIDTH) {
                left++;
            }

            int count = right - left + 1;

            if (count > bestCount) {
                bestCount = count;

                double windowMin = xPositions.get(left);
                double windowMax = xPositions.get(right);
                bestWindow = windowMax - windowMin;
                bestCenter = (windowMin + windowMax) / 2.0;
            }
        }
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("Best Ball Cluster Count", bestCount);
            telemetry.addData("Best Ball Cluster Center X", bestCenter);
            telemetry.addData("Best Ball Cluster Window size", bestWindow);
        }
        return bestCenter;
    }
    // Returns camera's field-relative position
    public Pose update(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
//<<<<<<< HEAD
//        Pose2D pose2d = pinpoint.getPosition();
//        double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
//        limelight.updateRobotOrientation(robotYaw + 180); // update for megatag 2
//
////        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
////        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//
//=======
//>>>>>>> c3e49e188f8fd4005824735df0a0a594ff98d47f
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            Pose3D botpose = result.getBotpose/*_MT2*/();//TODO: switch back if it's dog
            // pose3d, field relative, coords + orientation
//            RobotLog.dd("BotPose MT2", " " + botpose.getPosition().toUnit(DistanceUnit.INCH).toString() + "\n" + botpose.getOrientation().toString());
            double x = botpose.getPosition().toUnit(DistanceUnit.INCH).x; // x
            double y = botpose.getPosition().toUnit(DistanceUnit.INCH).y; // y
            double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS); // robot heading in radians
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("botpose heading");
                RobotLog.a("x y h" + x + " " + y + " " + heading);
            }
            return new Pose(x, y, heading);
        } else {
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                telemetry.addData("Limelight", "No Targets");
            }
            return null; // no target
        }
    }

    public double getTx() // degrees left or right of target
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            telemetry.addData("Target X", tx);
            return tx;
        }
        else return -1;
    }

    public double getTy() // degrees up or down relative to target
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            return ty;
        }
        else return -1;
    }

    public double getTa() // how big target is (0%->100%)
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ta = result.getTa();
            return ta;
        }
        else return -1;
    }

    public void updateTelemetry()
    {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // degrees left or right of target
            double ty = result.getTy(); // degrees up or down relative to target
            double ta = result.getTa(); // how big target is (0%->100% of img)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
    public double getDistanceToGoal() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        final double cameraHeightInches = 11.5;      // inches
        final double goalHeightInches = 29.5; // height of Apriltag (?)
        double limelightMountAngleDegrees = 0;

        double ty = result.getTy(); // vertical offset in degrees
        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double distanceFromLimelightToGoalInches = (goalHeightInches - cameraHeightInches)/Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }
}