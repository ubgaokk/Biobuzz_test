package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.Drivetrain;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lib.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


// FULL ROBOT CLASS: drive, shooter, intake, camera, beam break, stopper
public class Robot {
    private static Robot instance = null;

    // --------------SUBSYSTEMS---------------------------------------------------------------------
    public AprilFollower follower;
    Intake intake;
    private Shooter shooter;
    public Limelight camera;
//    BallDetector ballDetector;
    public Stopper stopper;
    List<LynxModule> allHubs;

    // ---------------BOOLEANS AND ATTRIBUTES-------------------------------------------------------
    double lastTime = 0; // Previous time, used to calculate loop time
    int driveSideSign = -1; // Alliance color changes forward vs backwards direction
    Pose txWorldPinpoint = new Pose(0,0,Math.PI); // Robot pose based on pinpoint in inverted FTC coordinates (not pedro coordinates)
    Vector robotVel = new Vector(0,0);
    Vector robotAccel = new Vector(0,0);
    double robotAngVel = 0;

    // Intake restart safety
    private boolean intakeStoppedWithoutAction = false;
    private boolean intakeDelayActive = false;
    private boolean startRelocalize = false;
    double relocalizeStartTime;
    private boolean cameraOff = false;
    ArrayList<Pose> poses = new ArrayList<>();
    // ---------------------------TIMERS------------------------------------------------------------
    ElapsedTime aprilTimer; //gonna use aprilTimer to also track loop times. Sorry, not sorry.
    ElapsedTime intakeTimer;
    ElapsedTime transferTimer;

    // --------------------------- ROBOT ENUMS------------------------------------------------------
    // Enum that manages shooter state machine
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        READY,
        SHOOTING,
    } private LaunchState launchState; // Instance

    //enum that manages intake state
    private enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    } private IntakeState intakeState; // Instance


    // Enum that switches between pinpoint and camera localization
    private enum DriveState {
        RELATIVE,
        ABSOLUTE, // Drive with AprilTag localization for absolute position on field
        ABSOLUTE2
    } private DriveState driveState; // Instance

    // Enum that stores the alliance color, accessible globally
    public enum Color {
        RED,
        BLUE
    } public Color color; // Instance

    // Enum that stores the opmode type, accessible globally
    public enum OpModeState {
        AUTO,
        AUTO_FAR,
        TELEOP,
        TELEOP_FAR
    } public OpModeState opModeState; // Instance

    // -------------------PUBLIC ROBOT CONSTRUCTOR--------------------------------------------------
    /**
     * Creates a robot instance with field-relative position starting at initialPose, and mirrors
     * shooting position and goal positions based on alliance color
     * @param initialPose
     * @param color
     */
    public Robot(Pose initialPose /*in pedro coords*/, Color color){
        // Poses are mirrored if BLUE
        this.color = color;
        if (this.color== Color.RED) {
            driveSideSign = -1;
            Positions.goalPos = Constants.goalPos;
            // Where the robot will shoot from:
            Positions.autoShotPose = Constants.autoShotPose;
            Positions.autoShotPose_Far = Constants.autoShotPose_Far;
            Positions.teleShotPose = PedroToFTC(Constants.teleShotPose);
            Positions.gatePose = Constants.gatePose;
            Positions.teleRelocPose_Close = Constants.teleRelocPose_Close;
            Positions.teleRelocPose_Far = Constants.teleRelocPose_Far;
            Positions.deltaH = Constants.deltaH;
            RobotLog.d("It's Red");
        }
        else if (this.color == Color.BLUE){
            driveSideSign = 1;
            Positions.goalPos = mirrorVector(Constants.goalPos);
            // Where the robot will shoot from:
            Positions.autoShotPose = Constants.autoShotPose.mirror();
            Positions.autoShotPose_Far = Constants.autoShotPose_Far.mirror();
            Positions.teleShotPose = PedroToFTC(Constants.teleShotPose.mirror());
            Positions.gatePose = Constants.gatePose.mirror();
            Positions.teleRelocPose_Close = Constants.teleRelocPose_Close;
            Positions.teleRelocPose_Far = Constants.teleRelocPose_Far;
            Positions.deltaH = Constants.deltaH;
            RobotLog.d("It's Blue");
        }
        else {
            RobotLog.d("What...");
        }
        // Robot's field relative pose, which starts at initialPose
        txWorldPinpoint = PedroToFTC(initialPose); //.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
//        voltageSensor = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
    }

    // --------------------SINGLETON CONSTRUCTOR AND GETTER-----------------------------------------
    /**
     * Create one instance of robot (singleton).
     * NOTE: ALL DEVICES MUST BE REINITIALIZED BEFORE EVERY OPMODE, THEY ARE NOT SAVED.
     */
    // Get singleton instance
    public static Robot getInstance(Pose initialPose, Color color){
        if (instance == null || instance.opModeState == OpModeState.TELEOP){
            RobotLog.d("making a new instance");
            instance = new Robot(initialPose, color);
        }
        else RobotLog.d("keeping the instance");
        return instance;
    }

    // Create new instance
    public static Robot startInstance(Pose initialPose, Color color){
        instance = new Robot(initialPose, color);
        return instance;
    }

    // Clear singleton instance
    public static void clearInstance(){
        instance = null;
    }


    // --------------------DRIVETRAIN CLASS OF ROBOT------------------------------------------------
    /**
     * Resets localizer using AprilTags
     * Extends Roadrunner MecanumDrive Class
     */
    public class AprilFollower{
        private final Follower base;

        public AprilFollower(Follower base) {
            this.base = base;
        }

        public void updateConstants() {
            base.updateConstants();
        }

        public void followPath(Path path, boolean holdEnd) {
            base.followPath(path, holdEnd);
        }

        public void startTeleOpDrive(boolean useBrakeMode) {
            base.startTeleOpDrive(useBrakeMode);
        }

        public void updatePose() {
            base.updatePose();
        }

        public Vector getCentripetalForceCorrection() {
            return base.getCentripetalForceCorrection();
        }

        public void activateAllPIDFs() {
            base.activateAllPIDFs();
        }

        public void setMaxPower(double set) {
            base.setMaxPower(set);
        }

        public void followPath(PathChain pathChain, boolean holdEnd) {
            base.followPath(pathChain, holdEnd);
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
            return base.atPose(pose, xTolerance, yTolerance);
        }

        public double getHeadingGoal(double t) {
            return base.getHeadingGoal(t);
        }

        public Path getCurrentPath() {
            return base.getCurrentPath();
        }

        public String[] debug() {
            return base.debug();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric, offsetHeading);
        }

        public PathChain getCurrentPathChain() {
            return base.getCurrentPathChain();
        }

        public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) {
            base.setDrivePIDFCoefficients(drivePIDFCoefficients);
        }

        public void activateCentripetal() {
            base.activateCentripetal();
        }

        public Vector getTeleopDriveVector() {
            return base.getTeleopDriveVector();
        }

        public VectorCalculator getVectorCalculator() {
            return base.getVectorCalculator();
        }

        public void followPath(PathChain pathChain) {
            base.followPath(pathChain);
        }

        public void updateCallbacks() {
            base.updateCallbacks();
        }

        public boolean isRobotStuck() {
            return base.isRobotStuck();
        }

        public Vector getDriveVector() {
            return base.getDriveVector();
        }

        public void holdPoint(BezierPoint point, double heading) {
            base.holdPoint(point, heading);
        }

        public void setTeleOpDrive(double forward, double strafe, double turn) {
            base.setTeleOpDrive(forward, strafe, turn);
        }

        public boolean getUseTranslational() {
            return base.getUseTranslational();
        }

        public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) {
            base.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients);
        }

        public void startTeleopDrive() {
            base.startTeleopDrive();
        }

        public boolean atParametricEnd() {
            return base.atParametricEnd();
        }

        public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
            return base.atPose(pose, xTolerance, yTolerance, headingTolerance);
        }

        public Vector getClosestPointTangentVector() {
            return base.getClosestPointTangentVector();
        }

        public int getChainIndex() {
            return base.getChainIndex();
        }

        public void updateDrivetrain() {
            base.updateDrivetrain();
        }

        public Pose getPointFromPath(double t) {
            return base.getPointFromPath(t);
        }

        public PathBuilder pathBuilder(PathConstraints constraints) {
            return base.pathBuilder(constraints);
        }

        public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) {
            base.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients);
        }

        public void updateErrorAndVectors() {
            base.updateErrorAndVectors();
        }

        public double getDistanceRemaining() {
            return base.getDistanceRemaining();
        }

        public void startTeleOpDrive() {
            base.startTeleOpDrive();
        }

        public double getTotalHeading() {
            return base.getTotalHeading();
        }

        public Vector getTranslationalCorrection() {
            return base.getTranslationalCorrection();
        }

        public ErrorCalculator getErrorCalculator() {
            return base.getErrorCalculator();
        }

        public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
            base.followPath(pathChain, maxPower, holdEnd);
        }

        public double getDriveError() {
            return base.getDriveError();
        }

        public Vector getTeleopHeadingVector() {
            return base.getTeleopHeadingVector();
        }

        public void update() {
            base.update();
        }

        public void pausePathFollowing() {
            base.pausePathFollowing();
        }

        public boolean getUseHeading() {
            return base.getUseHeading();
        }

        public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) {
            base.setTranslationalPIDFCoefficients(translationalPIDFCoefficients);
        }

        public PathPoint getClosestPose() {
            return base.getClosestPose();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
        }

        public void turnTo(double radians) {
            base.turnTo(radians);
        }

        public void activateTranslational() {
            base.activateTranslational();
        }

        public boolean isTurning() {
            return base.isTurning();
        }

        public double getCurrentPathNumber() {
            return base.getCurrentPathNumber();
        }

        public boolean getTeleopDrive() {
            return base.getTeleopDrive();
        }

        public PathPoint getPreviousClosestPose() {
            return base.getPreviousClosestPose();
        }

        public FollowerConstants getConstants() {
            return base.getConstants();
        }

        public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) {
            base.setHeadingPIDFCoefficients(headingPIDFCoefficients);
        }

        public void setXVelocity(double vel) {
            base.setXVelocity(vel);
        }

        public void updateVectors() {
            base.updateVectors();
        }

        public double getCentripetalScaling() {
            return base.getCentripetalScaling();
        }

        public Vector getHeadingVector() {
            return base.getHeadingVector();
        }

        public void setPose(Pose pose) {
            base.setPose(pose);
        }

        public PathBuilder pathBuilder() {
            return base.pathBuilder();
        }

        public double getClosestPointHeadingGoal() {
            return base.getClosestPointHeadingGoal();
        }

        public double getAngularVelocity() {
            return base.getAngularVelocity();
        }

        public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) {
            base.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients);
        }

        public void turn(double radians, boolean isLeft) {
            base.turn(radians, isLeft);
        }

        public Vector getTranslationalError() {
            return base.getTranslationalError();
        }

        public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
            base.setTeleOpDrive(forward, strafe, turn, offsetHeading);
        }

        public void setMaxPowerScaling(double maxPowerScaling) {
            base.setMaxPowerScaling(maxPowerScaling);
        }

        public double getPathCompletion() {
            return base.getPathCompletion();
        }

        public void setStartingPose(Pose pose) {
            base.setStartingPose(pose);
        }

        public boolean isTeleopDrive() {
            return base.isTeleopDrive();
        }

        public void setConstraints(PathConstraints pathConstraints) {
            base.setConstraints(pathConstraints);
        }

        public double getHeading() {
            return base.getHeading();
        }

        public void turnToDegrees(double degrees) {
            base.turnToDegrees(degrees);
        }

        public void activateHeading() {
            base.activateHeading();
        }

        public void resumePathFollowing() {
            base.resumePathFollowing();
        }

        public boolean isBusy() {
            return base.isBusy();
        }

        public boolean getUseDrive() {
            return base.getUseDrive();
        }

        public PoseTracker getPoseTracker() {
            return base.getPoseTracker();
        }

        public PathConstraints getConstraints() {
            return base.getConstraints();
        }

        public void setCentripetalScaling(double set) {
            base.setCentripetalScaling(set);
        }

        public void startTeleopDrive(boolean useBrakeMode) {
            base.startTeleopDrive(useBrakeMode);
        }

        public void holdPoint(Pose pose) {
            base.holdPoint(pose);
        }

        public double getCurrentTValue() {
            return base.getCurrentTValue();
        }

        public boolean getUseCentripetal() {
            return base.getUseCentripetal();
        }

        public void turnDegrees(double degrees, boolean isLeft) {
            base.turnDegrees(degrees, isLeft);
        }

        public void deactivateAllPIDFs() {
            base.deactivateAllPIDFs();
        }

        public Vector getCorrectiveVector() {
            return base.getCorrectiveVector();
        }

        public Pose getPose() {
            return base.getPose();
        }

        public boolean getFollowingPathChain() {
            return base.getFollowingPathChain();
        }

        public void setYVelocity(double vel) {
            base.setYVelocity(vel);
        }

        public Vector getAcceleration() {
            return base.getAcceleration();
        }

        public void followPath(Path path) {
            base.followPath(path);
        }

        public void updateErrors() {
            base.updateErrors();
        }

        public void breakFollowing() {
            base.breakFollowing();
        }

        public double getMaxPowerScaling() {
            return base.getMaxPowerScaling();
        }

        public double getDistanceTraveledOnPath() {
            return base.getDistanceTraveledOnPath();
        }

        public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
            base.holdPoint(point, heading, useHoldScaling);
        }

        public boolean isLocalizationNAN() {
            return base.isLocalizationNAN();
        }

        public Vector getVelocity() {
            return base.getVelocity();
        }

        public void setConstants(FollowerConstants constants) {
            base.setConstants(constants);
        }

        public PoseHistory getPoseHistory() {
            return base.getPoseHistory();
        }

        public void activateDrive() {
            base.activateDrive();
        }

        public Drivetrain getDrivetrain() {
            return base.getDrivetrain();
        }

        public double getHeadingError() {
            return base.getHeadingError();
        }

        // --------------------AprilTag Re-localization Method--------------------------------
        @SuppressLint("DefaultLocale")
        public boolean relocalize(Telemetry telemetry) {
            if (!startRelocalize) {
                poses.clear(); // poses cleared from list
                startRelocalize = true;
                relocalizeStartTime = aprilTimer.milliseconds();
            }
            // Update the pinpoint velocity estimate as normal
            base.update();
            robotVel = base.getVelocity();
            robotAngVel = base.getAngularVelocity();
            Vector vel = robotVel;
            double angVel = robotAngVel;
            txWorldPinpoint = PedroToFTC(follower.getPose());//.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);

            // ------------------------Exit conditions-----------------
            // Don't relocalize if robot moving too fast (motion blur)
            if (vel.getMagnitude() > 0.2 || Math.toDegrees(angVel) > 0.1 || aprilTimer.milliseconds() - relocalizeStartTime > 300) {
                startRelocalize = false;
                boolean success = false;
                if (poses.size() >= 2) { // not enough poses for avrging
                    success = applyAveragePose();
                    Positions.flywheelPowerOffset = 0;
                    Positions.hoodAngleManualOffset = 0;
                    Positions.turretAngleManualOffset = 0;
                }
                poses.clear();
                return success;
//                RobotLog.a("Too fast to relocalize or time reached!");
//                return true; // Returns true for successful relocalization
            }
            Pose tagPose = camera.update(telemetry);

            if (tagPose != null && !tagPose.roughlyEquals(new Pose(0,0,0),0.999)) {
                poses.add(tagPose); // adding valid pose
            }
            return false;
//
//            RobotLog.a("heading used for camera" + Math.toDegrees(txWorldPinpoint.getHeading()));
//            // Get camera's pose (field-relative) using AprilTag
//            Pose poseWorldTurret = camera.update(/*Math.toDegrees(txWorldPinpoint.getHeading()), */telemetry);
//            // If no pose was found, default to the pinpoint localizer
//            if (poseWorldTurret == null || poseWorldTurret.equals(new Pose(0,0,0))){
//                return false; // EXIT METHOD
//            }
//
//            // ------------------------Continue------------------------
//            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
////            poseWorldTurret = new Pose(poseWorldTurret.getX(), poseWorldTurret.getY(),poseWorldTurret.getHeading() + Math.PI/2);
//
//            // Transforming the turret pose into the robot (FIELD RELATIVE) pose--> Pose multiplication: pWR = pWT * pRT^-1.
//            // The calculation above is now unnecessary because the camera is mounted on the robot, not the turret
//            Pose poseWorldRobot = poseWorldTurret/*.(Robot.Constants.cameraPos.)*/;
//            txWorldPinpoint = poseWorldRobot;
//            // Set the localizer pose to the current field-relative pose based on AprilTag reading.
//            base.setPose(FTCToPedro(poseWorldRobot));//.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
//            Positions.flywheelPowerOffset = 0;
//            Positions.turretAngleManualOffset = 0;
//            // Telemetry displays robot position and heading information
//            RobotLog.a("counter ", counter); // display mt1 updates
//            RobotLog.a(String.format("Pose XY %6.1f %6.1f  (inch)",
//                    poseWorldRobot.getX(),
//                    poseWorldRobot.getY()));
//            RobotLog.a(String.format("Pose Heading %6.1f  (rad)",
//                    poseWorldRobot.getHeading()
//            ));
//            RobotLog.a(String.format("Pedro Pose XY %6.1f %6.1f  (inch)",
//                    FTCToPedro(poseWorldRobot).getX(),
//                    FTCToPedro(poseWorldRobot).getY()));
//            RobotLog.a(String.format("Pedro Pose Heading %6.1f  (rad)",
//                    FTCToPedro(poseWorldRobot).getHeading()
//            ));
//
//            return true; // Returns true for successful relocalization
        }
        private boolean applyAveragePose() {

            if (poses.size() < 2) return false; // not averaging small list

            // Compute mean
            double meanX = 0;
            double meanY = 0;
            double meanH = 0;
            double meanSin = 0;
            double meanCos = 0;

            for (Pose p : poses) {
                meanX += p.getX();
                meanY += p.getY();
                meanSin += Math.sin(p.getHeading());
                meanCos += Math.cos(p.getHeading());
                if (!Constants.MINIMIZE_TELEMETRY) {
                    RobotLog.a(String.format("Pose XY %6.1f %6.1f  (inch)",
                            p.getX(),
                            p.getY()
                    ));
                    RobotLog.a(String.format("Pose Heading %6.1f  (rad)",
                            p.getHeading()
                    ));
                }
            }

            // Averaging
            meanX /= poses.size();
            meanY /= poses.size();
            meanSin /= poses.size();
            meanCos /= poses.size();
            meanH = Math.atan2(meanSin, meanCos); // distance to apriltag

            // Remove outliers (distance > 2 inches from mean)
            ArrayList<Pose> filtered = new ArrayList<>();

            for (Pose p : poses) {
                double dx = p.getX() - meanX;
                double dy = p.getY() - meanY;
                if (Math.hypot(dx, dy) < 10.0) {
                    filtered.add(p);
                }
            }

            if (filtered.size() < 2) return false; // too random

            // Recompute average
            meanX = 0;
            meanY = 0;
            meanH = 0;
            meanSin = 0;
            meanCos = 0;

            for (Pose p : filtered) {
                meanX += p.getX();
                meanY += p.getY();
                meanSin += Math.sin(p.getHeading());
                meanCos += Math.cos(p.getHeading());
            }

            meanX /= filtered.size();
            meanY /= filtered.size();
            meanSin /= filtered.size();
            meanCos /= filtered.size();
            meanH = Math.atan2(meanSin, meanCos);

            Pose finalPose = new Pose(meanX, meanY, meanH);

            if (!Objects.isNull(finalPose) && !finalPose.roughlyEquals(new Pose(0,0,0),0.999)) {
                base.setPose(FTCToPedro(finalPose)); // gives pose to drivetrain
                txWorldPinpoint = finalPose;
            }
//            Positions.flywheelPowerOffset = 0;
//            Positions.turretAngleManualOffset = 0;
            if (!Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("Relocalized using " + filtered.size() + " samples");
            }
            return true;
        }
//        // ---------------------------Draw Robot Method---------------------------------------
        public void drawPose(@NonNull TelemetryPacket p){
            p.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(p.fieldOverlay(), PoseToPose2d(txWorldPinpoint));
            p.fieldOverlay().setStroke("#00FF00");
            Drawing.drawRobot(p.fieldOverlay(),PoseToPose2d(txWorldPinpoint).times(Constants.turretPos));
            p.fieldOverlay().setStroke("#0000FF");
            Drawing.drawRobot(p.fieldOverlay(),new Pose2d(Positions.goalPos.getXComponent(), Positions.goalPos.getYComponent(), 0));
        }

    } // END OF APRILDRIVE CLASS


    // ---------------------------AUTON INITIALIZATION----------------------------------------------
    // Initialize and set motors, shooter, timers
    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, OpModeState opModeState) {
        if (opModeState == OpModeState.AUTO_FAR){
            Positions.p = Constants.p_far;
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.deltaH = Constants.deltaH_far;
            Positions.shootPower = Constants.shootPower_Far;
            Positions.transferPower = Constants.transferPower_Far;
            Positions.flywheelPower = Constants.flywheelPower_Far;
        }
        else{
            Positions.p = Constants.p;
            Positions.goalPos = handleVector(Constants.goalPos);
            Positions.deltaH = Constants.deltaH;
            Positions.shootPower = Constants.shootPower;
            Positions.transferPower = Constants.transferPower;
            Positions.flywheelPower = Constants.flywheelPower;
        }
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        intake = new Intake(hardwareMap);
//        camera = new AprilTagLocalization2(hardwareMap);
        camera = new Limelight(hardwareMap);
        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
//        follower.setPose(FTCToPedro(txWorldPinpoint));//.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
//        follower = new AprilFollower(hardwareMap, txWorldPinpoint);
        shooter = new Shooter(hardwareMap);
//        ballDetector = new BallDetector(hardwareMap);
        stopper = new Stopper(hardwareMap);
//        redled = hardwareMap.get(LED.class, "shooterStatus_red");
//        greenled = hardwareMap.get(LED.class, "shooterStatus_green");
        RobotLog.a("txworldpinpoint: " +  txWorldPinpoint.getX() + ", " + txWorldPinpoint.getY() + ", " + txWorldPinpoint.getHeading());
        RobotLog.a("startPose: " + FTCToPedro(txWorldPinpoint).getX()+ ","  + FTCToPedro(txWorldPinpoint).getY()+ ", " + FTCToPedro(txWorldPinpoint).getHeading());
        RobotLog.a("goalPos: " + Positions.goalPos.toString());

        this.opModeState = opModeState;
        launchState = LaunchState.SPIN_UP;
        driveState = DriveState.RELATIVE;
        intakeState = IntakeState.IDLE;
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();

        Positions.drivePower = Constants.drivePower;
//        Positions.turretClip0 = Constants.turretClip0;
//        Positions.turretClip1 = Constants.turretClip1;
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        Constants.turretAngleOffset = 0;
        Constants.hoodAngleOffset = 0;

//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
//        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//        telemetry.addData("deltaH"," "+ Positions.deltaH);
//        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        telemetry.update();
    } // END OF INITIALIZATION FUNCTION

    // -----------------------TELEOP INITIALIZATION-------------------------------------------------
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
//        Constants.turretTargetRangeOffset = (Constants.turretLowAngle + Constants.turretHighAngle )/2.0;
        // Initialize subsystems
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        intake = new Intake(hardwareMap);
//        camera = new AprilTagLocalization2(hardwareMap);
        camera = new Limelight(hardwareMap);
        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        follower.setPose(FTCToPedro(txWorldPinpoint)); //.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
        shooter = new Shooter(hardwareMap);
//        ballDetector = new BallDetector(hardwareMap);
        stopper = new Stopper(hardwareMap);
//        turret = new Turret(hardwareMap);


        // Set enums
        opModeState = OpModeState.TELEOP;
        launchState = LaunchState.SPIN_UP;
        driveState = DriveState.RELATIVE;
        intakeState = IntakeState.IDLE;

        // Start timers
        aprilTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        transferTimer = new ElapsedTime();

        // Constants
        Positions.drivePower = Constants.drivePower_Tele;
//        Positions.turretClip0 = Constants.turretClip0;
//        Positions.turretClip1 = Constants.turretClip1; //CHANGE BACK
        Positions.flywheelPower = Constants.flywheelPower_Tele;
        Positions.flywheelPowerOffset = 0;
        Positions.turretAngleManualOffset = 0;
        Positions.hoodAngleManualOffset = 0;
        Constants.turretAngleOffset = 0;
        Constants.hoodAngleOffset = 0;

        // Logs and telemetry
//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//            telemetry.addData("goal pos", " "
//                    + Positions.goalPos.x + " "
//                    + Positions.goalPos.y);
//            telemetry.addData("auto shot pose far", " "
//                    + Positions.autoShotPose_Far.position.x + " "
//                    + Positions.autoShotPose_Far.position.y + " "
//                    + Positions.autoShotPose_Far.heading.toDouble());
//            telemetry.addData("tele shot pose", " "
//                    + Positions.teleShotPose.position.x + " "
//                    + Positions.teleShotPose.position.y + " "
//                    + Positions.teleShotPose.heading.toDouble());
//            telemetry.addData("initial pose", " "
//                    + txWorldPinpoint.position.x + " "
//                    + txWorldPinpoint.position.y + " "
//                    + txWorldPinpoint.heading.toDouble());
//            telemetry.addData("auto shot pose", " "
//                    + Positions.autoShotPose.position.x + " "
//                    + Positions.autoShotPose.position.y + " "
//                    + Positions.autoShotPose.heading.toDouble());
//            telemetry.addData("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("Status", "Initialized"); // IMPORTANT
//        if (!Constants.MINIMIZE_TELEMETRY) {
//            RobotLog.dd("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//            RobotLog.dd("auto shot pose", " " + Positions.autoShotPose.position.x + " " + Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//            RobotLog.dd("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " + Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//            RobotLog.dd("tele shot pose", " " + Positions.teleShotPose.position.x + " " + Positions.teleShotPose.position.y + " " + Positions.teleShotPose.heading.toDouble());
//            RobotLog.dd("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//            RobotLog.dd("deltaH", " " + Positions.deltaH);
//        }
//        telemetry.addData("goal pos", " " + Positions.goalPos.x + " " + Positions.goalPos.y);
//        telemetry.addData("auto shot pose far", " " + Positions.autoShotPose_Far.position.x + " " +  Positions.autoShotPose_Far.position.y + " " + Positions.autoShotPose_Far.heading.toDouble());
//        telemetry.addData("tele shot pose", " " + Positions.teleShotPose .position.x + " " +  Positions.teleShotPose .position.y + " " + Positions.teleShotPose .heading.toDouble());
//        telemetry.addData("initial pose", " " + txWorldPinpoint.position.x + " " + txWorldPinpoint.position.y + " " + txWorldPinpoint.heading.toDouble());
//        telemetry.addData("auto shot pose", " " + Positions.autoShotPose.position.x + " " +  Positions.autoShotPose.position.y + " " + Positions.autoShotPose.heading.toDouble());
//        telemetry.addData("deltaH"," "+ Positions.deltaH);
//        telemetry.addData("Status", "Initialized"); //IMPORTANT
//        RobotLog.d("dirve side sign"+driveSideSign);
//        telemetry.update();
    } // END OF INITIALIZATION FUNCTION

    public void startAuto(){
        aprilTimer.reset();
        intakeTimer.reset();
        transferTimer.reset();
    }
    // --------------------WHEN TELEOP STARTS-------------------------------------------------------
    public void startTeleop(){
        aprilTimer.reset();
        intakeTimer.reset();
        transferTimer.reset();
        follower.startTeleopDrive(true);
        stopper.open(); //comment
    }

    @Config
    // ---------------------------ROBOT CONSTANTS CLASS---------------------------------------------
    public static class Constants {
        public final static boolean MINIMIZE_TELEMETRY = false;
        public final static boolean friedFeed = true;
        public static double flywheelPower = 2.25; //2.25; //2.25; //2.15;//2.25; //2.45; //2.26; //2.150;
        public static double flywheelPower_Tele = 2.25; //2.25; //2.25; //2.15;//2.25; //2.45; //2.26; //2.100;
        public static double flywheelPower_Far = 1.700; //1.676; //1.68; //1.75; //1.2824; //1.697; //1.823; //1.655; //1.815; //1.655;
        public static double flywheelPowerFar_Tele = 1.815; //1.701;
        public static double hoodAngleOffset = 0;
        public final static double p = 0.69; //0.72; //0.75; //0.8; //0.7; //proportion along the trajectory
        public final static double p_far = 0.55; //0.52;
        public final static double deltaH = 30; //height of wall = 54, height of ball exit = 18 inch - 3 inch //32; //height of turret = 8.436535433
        public final static double deltaH_far = 30;
        // GOAL POSES
        public final static Vector goalPos = new Vector(Math.hypot(-58.3727-5.3,55.6425+5.3), Math.atan2(55.6425+5.3,-58.3727-5.3)); //distance to triangle center = 7.5 inch
        public final static Vector goalPos_far = new Vector(Math.hypot(-65,55.6425+9.3), Math.atan2(55.6425+9.3,-65)); //previously -65 //distance to triangle center = 7.5 inch
        // SHOT POSES
        private final static Pose autoShotPose = new Pose(56+2*(72-56), 82, Math.toRadians(-15)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//        public final static Pose autoShotPose = new Pose(-43,33,Math.toRadians(90)); //new Pose2d(-12,15,Math.toRadians(90));
        public final static Pose autoShotPose_Far = new Pose(56,12,Math.toRadians(120));
        public final static Pose teleShotPose = new Pose(82, 82, goalPos.getTheta()+3*Math.PI/2);
        public final static Pose teleShotPose_Far = new Pose(0,0,goalPos.getTheta()+Math.PI);
        private final static Pose gatePose = new Pose(30+2*(72-30), 62, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        private final static Pose gatePose2 = new Pose(14+2*(72-14), 61, Math.toRadians(35)); // Lowest (Third Set) of Artifacts from the Spike Mark.
        private final static Pose teleRelocPose_Close = new Pose(121.1673/*144-16.247*/,125.7631/* 123.0192*/,Math.toRadians(-140.8886/*-90-54.2651*/));
        private final static Pose teleRelocPose_Far = new Pose(9.7 /*10.3347*/ /*8.4107*/, 8.2/*9*//*71.5311,*//*12.1255*/,Math.toRadians(90/*180*/));
        // FLYWHEEL
//        public final static double spinUpTimeout = 2;
        public final static double kP = 0.074, /*0.052,*/ kI = 0, kD = 0.000, kF = 10 /*kF will be removed in our new version*/, kS = 0.34, /*1.94*/ kV = 0.00512, /*0.00552; //0.00450*/ kRadial = 0.03;
        public final static double humanFeedVel = -800 / 28.0 * Math.PI * 2; //in radps
        //STOPPER
        public final static double stopperScale0 = 0.33; //0.26; //0.12;
        public final static double stopperScale1 = 0.52; //0.32;
        //ARM
//        public final static double armScale0 = 0.04;
//        public final static double armScale1 = 0.27;
        // INTAKE
        public final static double iP = 20, iI = 3, iD = 0.000, iF = 0;
        public final static double tP = 20, tI = 3, tD = 0.000, tF = 0;
        public final static double transferPower = 1.0; //0.8; //1.0;
        public final static double transferPower_Far = 0.3; //0.35; //0.45; //0.25; //0.25;//0.25; //0.45; //0.67; //0.4; //0.67;
        public final static double throughPower = 0.65;
        public final static double slowIntakePower = 0.7;
        public final static double intakePower = 1.0;
        public final static double shootPower = 1.0 ;//0.8; //1.0;
        public final static double shootPower_Far = 0.4; //0.3; //0.5; //0.25;//0.45; //intake power ONLY for shooting
        public final static double farRampDelay = 0.6;
        public final static double farRampTime = 0.1; // smooth ramp duration
        public final static double farTransferMax = 1;
        public final static double farShootMax = 1;
        public final static double outtakePower = 1.0;
        public final static double intakeRestartDelay = 0.2; // seconds (tune 0.15–0.25)
//        public final static double intakePulseTime = 0.55;
//        public final static double outtakePulseTime = 0.15;
//        public final static double outtakePulseTime_Tele = 0.15;
//        public final static double intakeStabilizeTime = 0.4;
//        public final static double feedTime = 0.5; //0.5; //probably increase to 0.6
//        public final static double feedDownTime = 0.4; //less than feedUpTime probably
        // HOOD
        public final static double hoodLowAngle = 40 * Math.PI/180; //45 //65.36742754 * Math.PI/180; // the traj angle from horizonatla (rad) //75 //0;
        public final static double hoodHighAngle = (90-23)*Math.PI/180; //29 //24.47717215 * Math.PI/180; //30 //50*Math.PI/180; //the traj angle from horizontal 55; // Highest actual degree is 41
        public final static double hoodScale0 = 0;//0.2; //0.27;
        public final static double hoodScale1 = 1; //0.52; //0.65; //0.89; //1; //0.85; down
        public final static double hoodClip0 = 0.24 ;//0.465; //0.39; //.34; //0.30; //0.15; //0.1027; //0 ;//0.2 ; //0.27;
        public final static double hoodClip1 = 0.74; //0.97; //0.7821; //0.52; //0.65; //0.89; //1; //0.85; down
        // TURRET
        /**
         * turret 0 = 0.48
         * oldest ones also work
         * for backwards turret, add math.pi here
         */
        public final static double TURRET_FF_TIME = 0.2;
        public final static double turretGearRatio = 23.0/105;
        public static double turretAngleOffset = 0; //-69
        public final static Pose2d turretPos = new Pose2d(-0.7675,0,0);
//        public final static Pose cameraPos = new Pose(7.5, 2, Math.toRadians(0));
        public final static double turretHighAngle = Math.PI/2; //0; //Math.PI/2; //164.7*Math.PI/180; //220*Math.PI/180;; //355*Math.PI/180; // //140*Math.PI/180; // In rad, pos = 1
        public final static double turretLowAngle = -Math.PI/2; //0; //-175*Math.PI/180; //-40*Math.PI/180;; // //-208*Math.PI/180; // In rad (= old -330 deg)
//        public final static double turretTargetRangeOffset = turretHighAngle-Math.PI; //offset from (-pi,pi)
        // Offset from (-pi,pi) to (midpoint-pi, midpoint+pi), i.e. shift midpoint from 0 to new midpoint
        public final static double turretTargetRangeOffset = 0; //Math.PI/2; //(turretLowAngle + turretHighAngle )/2.0; //turretHighAngle-Math.PI;
        public final static double turretScale0 = 0.23; //0.40;  //0.33055555555555555; //0.218; //0; //0.25 ;//0; //0.11;
        public final static double turretScale1 = 0.75; //0.49; //0.66; //0.67; //1; //0.78; //0.86; //1;
        public final static double turretClip0 = 0.24; //PHYSICAL LIMIT: 0.20 //0;
//        public final static double turretClip0_tele = turretClip0; //turretScale0;
        public final static double turretClip1 = 0.71; //0.62; //PHYSICAL LIMIT: 0.72 //0.8;
//        public final static double turretClip1_tele = turretClip1; //turretScale1;
        public final static double driveRotationPower = 1.0;
        public final static double drivePower = 1.0;
        public final static double drivePower_Tele = 1.0;
        public final static double drivePower_Slow = 0.2;
    } // END OF CONSTANTS CLASS


    // ---------------------------POSITIONS CLASS---------------------------------------------------
    // These are mobile positions, which are mirrored depending on color. Also includes scalars that are toggled between different values. For better code readability and error proofing.
    public static class Positions{
        public static Vector goalPos = Constants.goalPos;
        // Where the robot will shoot from:
        public static Pose autoShotPose = Constants.autoShotPose;
        public static Pose autoShotPose_Far = Constants.autoShotPose_Far;
        public static Pose teleShotPose = Constants.teleShotPose;
        public static Pose gatePose = Constants.gatePose;
        public static Pose gatePose2 = Constants.gatePose2;
        public static PathChain intakeGate;
        public static Pose teleRelocPose_Close = Constants.teleRelocPose_Close;
        public static Pose teleRelocPose_Far = Constants.teleRelocPose_Far;
        public static Path scoreGate;
        public static double p = Constants.p;
        public static double deltaH = Constants.deltaH;
        public static double drivePower = Constants.drivePower;
        public static double transferPower = Constants.transferPower;
        public static double transferPowerOffset = 0;
        public static double shootPowerOffset = 0;
        public static double shootPower = Constants.shootPower;
//        public static double turretClip0 = Constants.turretClip0;
//        public static double turretClip1 = Constants.turretClip1;
        public static double flywheelPower = Constants.flywheelPower;
        public static double flywheelPowerOffset = 0;
        public static double turretAngleManualOffset = 0;
        public static double hoodAngleManualOffset = 0;
    } // END OF POSITIONS CLASS

    public void reportBatteryDraw(Telemetry telemetry){
        double flywheel = shooter.getFlywheel().FLYWHEEL_MASTER.getMotor().getCurrent(CurrentUnit.AMPS);
        double flywheel2 = shooter.getFlywheel().FLYWHEEL2.getMotor().getCurrent(CurrentUnit.AMPS);
        double intakE = intake.INTAKE.getMotor().getCurrent(CurrentUnit.AMPS);
        double transfer = intake.TRANSFER.getMotor().getCurrent(CurrentUnit.AMPS);
        telemetry.addData("flywheel 1 current", flywheel);
        telemetry.addData("flywheel 2 current", flywheel2);
        telemetry.addData("intake current", intakE);
        telemetry.addData("transfer current", transfer);
        telemetry.addData("total subsystem current", flywheel + flywheel2 + intakE + transfer);
    }
    public void clearBulkCache(){
        for(LynxModule hub  : allHubs){
            hub.clearBulkCache();
        }
    }
    // ---------------------------TELEOP FIELD-CENTRIC DRIVING--------------------------------------
    public void driveTele(double forward, double right, double rotate, boolean slowMode, boolean p2p1, boolean p2p2, boolean p2pEnded, Telemetry telemetry) {
        // Normal vs. slow mode
        if (p2pEnded){
//            follower.deactivateAllPIDFs();
            follower.startTeleopDrive(true);
            if (!Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("hiiu");
            }
        }
        if (!(p2p1||p2p2)) {
            if (slowMode) {
                Positions.drivePower = Constants.drivePower_Slow;
            } else {
                Positions.drivePower = Constants.drivePower_Tele;
            }
            follower.setTeleOpDrive(
                    forward * driveSideSign * Positions.drivePower,
                    right * driveSideSign * Positions.drivePower,
                    -rotate * Constants.driveRotationPower * Positions.drivePower,
                    true //true //false //true // Robot Centric][poiuytewq
            );
            if (!Constants.MINIMIZE_TELEMETRY) {
                telemetry.addLine("teleing");
                telemetry.addData("inner forward", forward * driveSideSign * Positions.drivePower);
            }
//            follower.activateHeading();
//            follower.startTeleOpDrive();
        }
        else if (p2p1){
            follower.holdPoint(FTCToPedro(Positions.teleShotPose));// new Pose(14+2*(72-14), 58, Math.toRadians(30))/*or a defined shooting pose*/);
            telemetry.addLine("p2ping");
        }
        else if (p2p2){
            follower.holdPoint( new Pose(14+2*(72-14), 58, Math.toRadians(30))/*or a defined shooting pose*/);
        }
    } // END OF DRIVING FUNCTION


    // ---------------------------SHOOTER METHOD----------------------------------------------------
    // Calculates and sets hood angle and flywheel RPM. Includes shooter state manager.

    //TODO: still need to incorporate a timeout, and manual override
    //TODO: also move shot and setRobotPose
    //TODO: removed ballDetector, as keeping shooter spun up is fine. verify it
    public void updateShooter(Telemetry telemetry, boolean shootRequested, boolean spinup, boolean humanFeed, boolean lock, boolean idle, boolean moveShot, Pose setRobotPose, double flywheelChange, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        //basically will decide whether the shooter should be spinning or not, modifying some params before passing into shooter update
        boolean shoot = true;
        switch (launchState) {
            case IDLE: //flywheel is powered off and hood/turret are not moving
                //if sees ball at top or intaking is off or transferring or force shot ready ==> SPIN_UP
                shoot = false;
                if (/*ballDetector.ballPresent(telemetry) ||*/ /*intakeState == IntakeState.TRANSFERRING*/ shootRequested || spinup/* || humanFeed*/) {
                    launchState = LaunchState.SPIN_UP;
                }
//                redled.on();
//                greenled.off();
                break;
            case SPIN_UP: //flywheel is reaching the desired speed and hood/turret are moving to pos
                //if flywheel speed and hood/turret angles are correct ==> SHOOTING
                if (shooter.isAimed()) {
                    launchState = LaunchState.READY;
                }
                //if idle requested ==> IDLE
                else if (idle) {
                    launchState = LaunchState.IDLE;
                }
//                redled.on();
//                greenled.on();
                break;
            case READY: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (!shooter.isAimed()) {
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle) {
                    launchState = LaunchState.IDLE;
                } else if (shootRequested) {
//                    stopper.open(); //better to only have it closed during intake. otherwise open stopper might cause some delay
                    launchState = LaunchState.SHOOTING;
                }
//                redled.off();
//                greenled.on();
                break;
            case SHOOTING: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (!shooter.isAimed()) {
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle /*|| (!ballDetector.ballPresent(telemetry) && !shootRequested)*/) {
//                    stopper.close();
                    launchState = LaunchState.IDLE;
                }
//                redled.off();
//                greenled.on();
                break;
        }
        Pose robotPose = txWorldPinpoint;
        if (lock) {
            robotPose = setRobotPose;
        } else if (moveShot) {
            robotPose = shooter.adjustMovementPose(txWorldPinpoint, robotVel, robotAngVel);
            telemetry.addLine("CURRENTLY SHOOTING WHILE MOVING");
        }
        shooter.update(shoot, humanFeed, robotPose, flywheelChange > 0.95, flywheelChange < -0.95, hoodUp, hoodDown, turretLeft, turretRight);
        shooter.sendTelemetry(telemetry);
    }

    //TODO: still need to incorporate a timeout, and manual override
    //TODO: also move shot and setRobotPose
    //TODO: removed ballDetector, as keeping shooter spun up is fine. verify it
    public void updateShooter2(Telemetry telemetry, boolean shootRequested, boolean spinup, boolean humanFeed, boolean lock, boolean idle, boolean moveShot, Pose setRobotPose, double flywheelChange, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        //basically will decide whether the shooter should be spinning or not, modifying some params before passing into shooter update
        boolean shoot = true;
        switch (launchState) {
            case IDLE: //flywheel is powered off and hood/turret are not moving
                //if sees ball at top or intaking is off or transferring or force shot ready ==> SPIN_UP
                shoot = false;
                if (/*ballDetector.ballPresent(telemetry) ||*/ /*intakeState == IntakeState.TRANSFERRING*/ shootRequested || spinup/* || humanFeed*/) {
                    launchState = LaunchState.SPIN_UP;
                }
//                redled.on();
//                greenled.off();
                break;
            case SPIN_UP: //flywheel is reaching the desired speed and hood/turret are moving to pos
                //if flywheel speed and hood/turret angles are correct ==> SHOOTING
                if (shooter.isAimed()) {
                    launchState = LaunchState.READY;
                }
                //if idle requested ==> IDLE
                else if (idle) {
                    launchState = LaunchState.IDLE;
                }
//                redled.on();
//                greenled.on();
                break;
            case READY: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (!shooter.isAimed()) {
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle) {
                    launchState = LaunchState.IDLE;
                } else if (shootRequested) {
//                    stopper.open(); //better to only have it closed during intake. otherwise open stopper might cause some delay
                    launchState = LaunchState.SHOOTING;
                }
//                redled.off();
//                greenled.on();
                break;
            case SHOOTING: //ready to shoot
                //if flywheel speed and hood/turret angles are incorrect ==> SPIN_UP
                if (!shooter.isAimed()) {
                    launchState = LaunchState.SPIN_UP;
                }
                //if idle requested or (no ball at top AND not transferring) ==> IDLE. idle request should be stronger. but automatic idle should be weaker.
                else if (idle /*|| (!ballDetector.ballPresent(telemetry) && !shootRequested)*/) {
//                    stopper.close();
                    launchState = LaunchState.IDLE;
                }
//                redled.off();
//                greenled.on();
                break;
        }
        Pose robotPose = txWorldPinpoint;
        if (lock) {
            robotPose = setRobotPose;
            moveShot = false;
        }
        else if (moveShot) {
//            robotPose = shooter.adjustMovementPose(txWorldPinpoint, robotVel, robotAngVel);
            telemetry.addLine("CURRENTLY SHOOTING WHILE MOVING");
        }
        shooter.update2(shoot, humanFeed, moveShot, moveShot, robotPose, robotVel, robotAccel,robotAngVel,flywheelChange > 0.95, flywheelChange < -0.95, hoodUp, hoodDown, turretLeft, turretRight);
//        shooter.update(shoot, humanFeed, robotPose, flywheelChange > 0.95, flywheelChange < -0.95, hoodUp, hoodDown, turretLeft, turretRight);
        shooter.sendTelemetry(telemetry);
    }



    // ---------------------------INTAKE METHOD----------------------------------------------------
    //TODO: figure out ball counting
    //TODO: figure out how balls will rest inside the ramp
    public void updateIntake(boolean in, boolean out, boolean stopIn, boolean shoot, Telemetry telemetry) {
        // ---------------START OF SHOOTER STATE MANAGER----------------------
        switch (intakeState) {
            case IDLE: // ------------------------------------------------------
                if (shoot) {
                    //TODO: think about removing this condition. flywheel may slow down after every shot. but this is only starting the shots. so idk
//                    if (/*launchState == LaunchState.SHOOTING*/ shooter.isAimed()) { //TODO: 0.5s timeout here later
                    intakeStoppedWithoutAction = false;
                    intakeDelayActive = false;

                    intakeState = IntakeState.TRANSFERRING;
                    transferTimer.reset();
                    intake.shoot();

//                    }
                } else if (in) {
//                    stopper.close();
//                    intake.intake();
//                    intakeState = IntakeState.INTAKING;
                    // If restarting after stopping without shoot/outtake
                    if (intakeStoppedWithoutAction) {
                        stopper.close();                    // move stopper first
                        intakeTimer.reset();
                        intakeDelayActive = true;
                        intakeStoppedWithoutAction = false; // consume the flag
                        intakeState = IntakeState.INTAKING;
                    }
                    else {
                        stopper.close();
                        intake.intake();
                        intakeState = IntakeState.INTAKING;
                    }
                } else if (out) {
                    intakeStoppedWithoutAction = false;
                    intakeDelayActive = false;

                    intake.outtake();
                    intakeState = IntakeState.OUTTAKING;
                }
//                if (4) {
//                    intake.outtake();
//                    intakeState = IntakeState.OUTTAKING;
//                }
                break; // END OF IDLE STATE

            case INTAKING:
                // If delay active, wait before spinning motor
                if (intakeDelayActive) {
                    if (intakeTimer.seconds() > Constants.intakeRestartDelay) {
                        intakeDelayActive = false;
                        intake.intake();
                    }
                }
                if (stopIn){
                    stopper.open();
                    intake.stop();

                    intakeStoppedWithoutAction = true;  // <-- key flag
                    intakeState = IntakeState.IDLE;
                }
                //TODO: remove this
//                if (ballDetector.ballPresent(telemetry)){ //ball present at top. or 3
//                    intake.stop();
//                    intakeState = IntakeState.IDLE;
//                }
                //TODO: add outtake and shoot requests
                else if (shoot) {
                    //TODO: think about removing this. flywheel may slow down after every shot. but this is only starting the shots. so idk
                    if (/*launchState == LaunchState.SHOOTING*/ true /*shooter.isAimed()*/) { //TODO: 0.5s timeout here later
                        intakeStoppedWithoutAction = false;
                        stopper.open();
                        intake.shoot();
                        transferTimer.reset();
                        intakeState = IntakeState.TRANSFERRING;
                    }
                }
                else if (out) {
                    intakeStoppedWithoutAction = false;
                    stopper.open();
                    intake.outtake();
                    intakeState = IntakeState.OUTTAKING;
                }
                break;
            case OUTTAKING:
                if (!out){
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
                break;
            case TRANSFERRING: // FEED BALL------------------------------------
                double t = transferTimer.seconds();

                if (opModeState == OpModeState.AUTO_FAR || opModeState == OpModeState.TELEOP_FAR) {

                    double shootPower = Positions.shootPower;
                    double transferPower = Positions.transferPower;

                    if (t > Constants.farRampDelay) {

                        double rampProgress = Math.min(
                                (t - Constants.farRampDelay) / Constants.farRampTime,
                                1.0
                        );

                        shootPower = shootPower +
                                rampProgress * (Constants.farShootMax - shootPower);

                        transferPower = transferPower +
                                rampProgress * (Constants.farTransferMax - transferPower);
                    }

                    intake.INTAKE.getMotor().setVelocity(shootPower * 1900);
                    intake.TRANSFER.getMotor().setVelocity(transferPower * 1900);
                }
                if (!shoot){
                    intake.stop();
                    intakeState = IntakeState.IDLE;
                }
                break;
        }  // --------------------END OF STATE MANAGER--------------------------------
        if (!Constants.MINIMIZE_TELEMETRY) {
            intake.telemetry(telemetry);
            telemetry.addData("transfer power", Positions.transferPower);
        }
//        telemetry.update();
    } // -------------------------------END OF UPDATE SHOOTER---------------------------------------

    // ----------------------------------LOCALIZATION CLASS-----------------------------------------
    /**
     * Returns field-relative robot pose (calculated using turret pose), or returns Pinpoint-recorded
     * pose if no AprilTag detections. Also displays pose information on telemetry.
     */
    public void updateFollower(boolean relocalize, boolean relocalize2, boolean toggleCamera, Telemetry telemetry){
        if (toggleCamera){
            cameraOff = !cameraOff;
        }
        switch (driveState){
            case RELATIVE:
                follower.update();
                txWorldPinpoint = PedroToFTC(follower.getPose());//.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
                robotVel = follower.getVelocity();
                robotVel.rotateVector(Math.PI/2);
                robotAngVel = follower.getAngularVelocity();
                robotAccel = follower.getAcceleration();
                robotAccel.rotateVector(Math.PI/2);
//                if (aprilTimer.seconds() > 10){
//                    driveState = DriveState.ABSOLUTE;
//                }
                if (relocalize /*|| aprilTimer.seconds() > 10*/ && !cameraOff) driveState = DriveState.ABSOLUTE;
                if (relocalize2) driveState = DriveState.ABSOLUTE2;
                break;
            case ABSOLUTE:
                boolean over = follower.relocalize(telemetry); // relocalization w/ apriltags
                if (over) {aprilTimer.reset();
                    driveState = DriveState.RELATIVE;}
//                txWorldPinpoint = PedroToFTC(follower.getPose());//.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
                break;
            case ABSOLUTE2:
                follower.update();
                Pose newPose = handlePose(Positions.teleRelocPose_Close);
                if (opModeState == OpModeState.TELEOP_FAR){
                    newPose = handlePose(Positions.teleRelocPose_Far);
                }
//                Pose newPose = new Pose(71.5311,8.4107,Math.toRadians(90));
                follower.setPose(newPose);//.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
                txWorldPinpoint = PedroToFTC(newPose);
                Positions.flywheelPowerOffset = 0;
//                Constants.hoodAngleOffset = 0;
//                Constants.turretAngleOffset = 0;
                Positions.turretAngleManualOffset = 0;
                Positions.hoodAngleManualOffset = 0;
                cameraOff = true;
                aprilTimer.reset();
                driveState = DriveState.RELATIVE;
                break;
        }
        camera.updateHeading(Math.toDegrees(txWorldPinpoint.getHeading()));
        double curTime = aprilTimer.milliseconds();
        telemetry.addData("time since last relocalization (s)", curTime/1000.0); //IMPORTANT
        if (cameraOff) telemetry.addLine("CAMERA OFF"); //IMPORTANT
        telemetry.addData("follower x", follower.getPose().getX()); //ALL IMPORTANT
        telemetry.addData("follower y", follower.getPose().getY());
        telemetry.addData("follower heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        lastTime = curTime;
        if (!Constants.MINIMIZE_TELEMETRY) {
            telemetry.addData("loop time (ms)",curTime-lastTime);
            telemetry.addData("world x", txWorldPinpoint.getX()); //ALL IMPORTANT
            telemetry.addData("world y", txWorldPinpoint.getY());
            telemetry.addData("world heading (deg)", Math.toDegrees(txWorldPinpoint.getHeading()));
            telemetry.addData("robot vel (in/s)", robotVel.toString());
            telemetry.addData("robot ang vel (in/s)", robotAngVel);
        }
    } //----------------------------END OF LOCALIZATION CLASS---------------------------------------

    // ---------------------------------------AIMING------------------------------------------------
    public boolean setGoalTarget(){
        double x = txWorldPinpoint.getX();
        boolean close = x < 15;
        if (close){
            Positions.goalPos = handleVector(Constants.goalPos);
            Positions.p = Constants.p;
            Positions.flywheelPower = Constants.flywheelPower_Tele;
            Positions.deltaH = Constants.deltaH;
//            Positions.teleShotPose = handlePose(Constants.teleShotPose);
            Positions.shootPower = Constants.shootPower;
            Positions.transferPower=  Constants.transferPower;
            opModeState = OpModeState.TELEOP;
        }
        else { //chagne to be better at far
            Positions.goalPos = handleVector(Constants.goalPos_far);
            Positions.p = Constants.p_far;
            Positions.flywheelPower = Constants.flywheelPower_Far;
            Positions.deltaH = Constants.deltaH_far;
//            Positions.teleShotPose = handlePose(Constants.teleShotPose_Far);
            Positions.shootPower = Constants.shootPower_Far;
            Positions.transferPower = Constants.transferPower_Far;
            opModeState = OpModeState.TELEOP_FAR;
        }
        return close;
    }

    // ------------------------Update pose variable with robot's current pose-----------------------
    public void setPose(Pose pose){
        txWorldPinpoint = pose;
    }


    // --------------------------------HELPER METHODS-----------------------------------------------
    // Mirror a vector
    public Vector mirrorVector(Vector vector){
//        vector.setOrthogonalComponents(vector.getXComponent(), -vector.getYComponent());
        return new Vector(vector.getMagnitude(), -vector.getTheta());
    }
    // If blue, mirror the pose
    public Pose handlePose(Pose pose){
        if (color == Color.BLUE){
            return pose.mirror();
        }
        return pose;
    }
    // If blue, mirror the vector
    public Vector handleVector(Vector vector){
        if (color == Color.BLUE){
            return mirrorVector(vector);
        }
        return vector;
    }
    public Pose FTCToPedro(Pose pose){
        Pose rotatedPose = pose.rotate(-Math.PI / 2, true);
        return rotatedPose.plus(new Pose(72, 72));
    }
    public Pose PedroToFTC(Pose pose){
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(Math.PI / 2, true);
    }
    public Pose2d PoseToPose2d(Pose pose){
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }
    public Pose Pose2dToPose(Pose2d pose2d){
        return new Pose(pose2d.position.x, pose2d.position.y, pose2d.heading.toDouble());
    }

    public static void display(char[] arr){
        String message = "";
        for (char c : arr) message += c + " ";
        RobotLog.d(message);
    }
    public static void display(int[] arr){
        String message = "";
        for (int j : arr) message += j + " ";
        RobotLog.d(message);
    }
}