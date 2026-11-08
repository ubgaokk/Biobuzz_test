package org.firstinspires.ftc.moon.drive;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import com.pedropathing.math.Vector;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.Drawing;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * MoonRobot - Drive system extracted from DECODE V2
 * Package: org.firstinspires.ftc.moon.drive
 */
//@Config
public class MoonRobot {
    // ==================== CONSTANTS (exact copy from DECODE V2) ====================
    public static class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                .forwardZeroPowerAcceleration(-35.356022741184866)
                .lateralZeroPowerAcceleration(-69)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0))
                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0, 0.00001, 0.6, 0.0))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0.0, 0.000005, 0.6, 0.0))
                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0))
                .centripetalScaling(0.0005)
                .mass(12.066);

        public static com.pedropathing.ftc.drivetrains.MecanumConstants driveConstants = new com.pedropathing.ftc.drivetrains.MecanumConstants()
                .maxPower(1)
                .xVelocity(68)
                .yVelocity(55.8)
                .rightFrontMotorName("W_FR")
                .rightRearMotorName("W_BR")
                .leftRearMotorName("W_BL")
                .leftFrontMotorName("W_FL")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

        public static com.pedropathing.ftc.localization.constants.PinpointConstants localizerConstants = new com.pedropathing.ftc.localization.constants.PinpointConstants()
                .forwardPodY(0.945)
                .strafePodX(1.488)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1.5);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new com.pedropathing.ftc.FollowerBuilder(followerConstants, hardwareMap)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .mecanumDrivetrain(driveConstants)
                    .build();
        }

        public static double drivePower_Tele = 1.0;
        public static double drivePower_Slow = 0.3;
        public static double driveRotationPower = 0.8;
        public static final double FIELD_OFFSET = 72;

        public static boolean MINIMIZE_TELEMETRY = false;
    }

    // ==================== SINGLETON ====================
    private static MoonRobot instance = null;

    public enum Color { RED, BLUE }
    public enum OpModeState { AUTO, TELEOP }

    public Color color = Color.BLUE;
    public OpModeState opModeState = OpModeState.TELEOP;
    public AprilFollower follower;
    private List<LynxModule> allHubs;

    private int driveSideSign = -1;
    private double drivePower = Constants.drivePower_Tele;

    private Pose txWorldPinpoint = new Pose(0, 0, Math.PI);
    private com.qualcomm.robotcore.util.ElapsedTime aprilTimer;

    ArrayList<Pose> poses = new ArrayList<>();

    // ==================== CONSTRUCTOR ====================
    public MoonRobot(Pose initialPose /*in pedro coords*/, Color color) {
        this.color = color;
        if (this.color == Color.RED) {
            driveSideSign = -1;
        } else if (this.color == Color.BLUE) {
            driveSideSign = 1;
        }
        this.txWorldPinpoint = PedroToFTC(initialPose);
    }

    public static MoonRobot getInstance() {
        return instance;
    }

    public static MoonRobot createInstance(Pose initialPose /*in pedro coords*/, Color color) {
        if (instance == null || instance.opModeState == OpModeState.TELEOP) {
            instance = new MoonRobot(initialPose, color);
        }
        return instance;
    }

    public static void clearInstance() {
        instance = null;
    }

    // ==================== INITIALIZATION ====================
    public void initTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new AprilFollower(Constants.createFollower(hardwareMap));
//        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        follower.setPose(FTCToPedro(txWorldPinpoint));

        aprilTimer = new com.qualcomm.robotcore.util.ElapsedTime();
        drivePower = Constants.drivePower_Tele;

        RobotLog.d("MoonRobot teleop initialized for " + color);
    }

    public void initAuto(HardwareMap hardwareMap, Telemetry telemetry, OpModeState opModeState) {
        this.opModeState = opModeState;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new AprilFollower(Constants.createFollower(hardwareMap));
//        follower = new AprilFollower(org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap));
        follower.setPose(FTCToPedro(txWorldPinpoint)); // Kanke: Need to recover and check
//        follower.setPose(new Pose(72, 72, Math.PI));
        aprilTimer = new com.qualcomm.robotcore.util.ElapsedTime();
        drivePower = Constants.drivePower_Tele;

        RobotLog.d("MoonRobot auto initialized for " + color);
    }

    public void startTeleop() {
        follower.startTeleopDrive(true);
    }

    // ==================== DRIVE CONTROL ====================
    public void driveTele(double forward, double right, double rotate, boolean slowMode,
                          boolean p2p1, boolean p2p2, boolean p2pEnded,
                          Telemetry telemetry) {
        if (p2pEnded) {
            follower.startTeleopDrive(true);
        }

        if (!(p2p1 || p2p2)) {
            if (slowMode) {
                drivePower = Constants.drivePower_Slow;
            } else {
                drivePower = Constants.drivePower_Tele;
            }

            follower.setTeleOpDrive(
                    forward * driveSideSign * drivePower,
                    right * driveSideSign * drivePower,
                    -rotate * Constants.driveRotationPower * drivePower,
                    true
            );
            telemetry.addData("inner forward", forward * driveSideSign * drivePower);
            telemetry.addData("inner right", right * driveSideSign * drivePower);
        } else if (p2p1) {
            follower.holdPoint(FTCToPedro(new Pose(58, 58, Math.toRadians(30))));
        } else if (p2p2) {
            follower.holdPoint(new Pose(58, 58, Math.toRadians(30)));
        }
    }

    public void drive(double forward, double strafe, double rotate, boolean slow) {
        if (slow) {
            drivePower = Constants.drivePower_Slow;
        } else {
            drivePower = Constants.drivePower_Tele;
        }

        follower.setTeleOpDrive(
                forward * driveSideSign * drivePower,
                strafe * driveSideSign * drivePower,
                -rotate * Constants.driveRotationPower * drivePower,
                false
        );
    }

    // ==================== PATH FOLLOWING ====================
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    // ==================== POSE MANAGEMENT ====================
    public void updatePose() {
        follower.updatePose();
    }

    public Pose getPose() {
        return PedroToFTC(follower.getPose());
    }

    public Pose getRawPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        txWorldPinpoint = pose;
        follower.setPose(FTCToPedro(pose));
    }

    // ==================== UTILITY ====================
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return follower.atPose(pose, xTolerance, yTolerance, headingTolerance);
    }

    public boolean atParametricEnd() {
        return follower.atParametricEnd();
    }

    public Path getCurrentPath() {
        return follower.getCurrentPath();
    }

    public PathChain getCurrentPathChain() {
        return follower.getCurrentPathChain();
    }

    public void holdPoint(BezierPoint point, double heading) {
        follower.holdPoint(point, heading);
    }

//    public void holdPoint(Pose point, double heading) {
//        follower.holdPoint(point, heading);
//    }

    public void updateFollower(boolean relocalize, boolean cameraOff, boolean shootRequest,
                               Telemetry telemetry) {
        follower.update();
        if (relocalize) {
            RobotLog.d("Relocalize triggered");
        }
    }

    public void drawPose(TelemetryPacket packet) {
        // Pose visualization placeholder
    }

    // ==================== COORDINATE CONVERSION ====================
    public static Pose FTCToPedro(Pose pose) {
        Pose rotated = pose.rotate(-Math.PI / 2, true);
        return rotated.plus(new Pose(Constants.FIELD_OFFSET, Constants.FIELD_OFFSET));
    }

    public Pose PedroToFTC(Pose pedroPose) {
        Pose normalized = pedroPose.minus(new Pose(Constants.FIELD_OFFSET, Constants.FIELD_OFFSET));
        return normalized.rotate(Math.PI / 2, true);
    }

    public Pose mirrorPose(Pose pose) {
        return new Pose(pose.getX(), -pose.getY() + 2 * Constants.FIELD_OFFSET, pose.getHeading());
    }

    // ==================== APRILFOLLOWER WRAPPER ====================
//    public class AprilFollower {
//        private final Follower base;
//
//        public AprilFollower(Follower base) {
//            this.base = base;
//        }
//
//        public void setPose(Pose pose) {
//            base.setPose(pose);
//        }
//
//        public Pose getPose() {
//            return base.getPose();
//        }
//
//        public void startTeleopDrive() {
//            base.startTeleopDrive();
//        }
//
//        public void startTeleopDrive(boolean useBrakeMode) {
//            base.startTeleopDrive(useBrakeMode);
//        }
//
//        public void setTeleOpDrive(double forward, double strafe, double turn) {
//            base.setTeleOpDrive(forward, strafe, turn);
//        }
//
//        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
//            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
//        }
//
//        public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
//            base.setTeleOpDrive(forward, strafe, turn, isRobotCentric, offsetHeading);
//        }
//
//        public void followPath(Path path, boolean holdEnd) {
//            base.followPath(path, holdEnd);
//        }
//
//        public void followPath(PathChain pathChain, boolean holdEnd) {
//            base.followPath(pathChain, holdEnd);
//        }
//
//        public void followPath(PathChain pathChain) {
//            base.followPath(pathChain);
//        }
//
//        public void updatePose() {
//            base.updatePose();
//        }
//
//        public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
//            return base.atPose(pose, xTolerance, yTolerance);
//        }
//
//        public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
//            return base.atPose(pose, xTolerance, yTolerance, headingTolerance);
//        }
//
//        public boolean atParametricEnd() {
//            return base.atParametricEnd();
//        }
//
//        public Path getCurrentPath() {
//            return base.getCurrentPath();
//        }
//
//        public PathChain getCurrentPathChain() {
//            return base.getCurrentPathChain();
//        }
//
//        public Vector getDriveVector() {
//            return base.getDriveVector();
//        }
//
//        public Vector getTeleopDriveVector() {
//            return base.getTeleopDriveVector();
//        }
//
//        public String[] debug() {
//            return base.debug();
//        }
//
//        public void holdPoint(BezierPoint point, double heading) {
//            base.holdPoint(point, heading);
//        }
//        public void holdPoint(Pose pose) {
//            base.holdPoint(pose);
//        }
//
////        public void holdPoint(Pose point, double heading) {
////            base.holdPoint(point, heading);
////        }
//
//        public void updateDrivetrain() {
//            base.updateDrivetrain();
//        }
//
//        public boolean isRobotStuck() {
//            return base.isRobotStuck();
//        }
//
//        public Pose getPointFromPath(double t) {
//            return base.getPointFromPath(t);
//        }
//
//        public Vector getClosestPointTangentVector() {
//            return base.getClosestPointTangentVector();
//        }
//
//        public int getChainIndex() {
//            return base.getChainIndex();
//        }
//
//        public void setMaxPower(double set) {
//            base.setMaxPower(set);
//        }
//
//        public void updateConstants() {
//            base.updateConstants();
//        }
//    }
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
//        @SuppressLint("DefaultLocale")
//        public boolean relocalize(Telemetry telemetry) {
//            if (!startRelocalize) {
//                poses.clear(); // poses cleared from list
//                startRelocalize = true;
//                relocalizeStartTime = aprilTimer.milliseconds();
//            }
//            // Update the pinpoint velocity estimate as normal
//            base.update();
//            robotVel = base.getVelocity();
//            robotAngVel = base.getAngularVelocity();
//            Vector vel = robotVel;
//            double angVel = robotAngVel;
//            txWorldPinpoint = PedroToFTC(follower.getPose());//.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
//
//            // ------------------------Exit conditions-----------------
//            // Don't relocalize if robot moving too fast (motion blur)
//            if (vel.getMagnitude() > 0.2 || Math.toDegrees(angVel) > 0.1 || aprilTimer.milliseconds() - relocalizeStartTime > 300) {
//                startRelocalize = false;
//                boolean success = false;
//                if (poses.size() >= 2) { // not enough poses for avrging
//                    success = applyAveragePose();
//                    Robot.Positions.flywheelPowerOffset = 0;
//                    Robot.Positions.hoodAngleManualOffset = 0;
//                    Robot.Positions.turretAngleManualOffset = 0;
//                }
//                poses.clear();
//                return success;
//    //                RobotLog.a("Too fast to relocalize or time reached!");
//    //                return true; // Returns true for successful relocalization
//            }
//            Pose tagPose = camera.update(telemetry);
//
//            if (tagPose != null && !tagPose.roughlyEquals(new Pose(0,0,0),0.999)) {
//                poses.add(tagPose); // adding valid pose
//            }
//            return false;
//    //
//    //            RobotLog.a("heading used for camera" + Math.toDegrees(txWorldPinpoint.getHeading()));
//    //            // Get camera's pose (field-relative) using AprilTag
//    //            Pose poseWorldTurret = camera.update(/*Math.toDegrees(txWorldPinpoint.getHeading()), */telemetry);
//    //            // If no pose was found, default to the pinpoint localizer
//    //            if (poseWorldTurret == null || poseWorldTurret.equals(new Pose(0,0,0))){
//    //                return false; // EXIT METHOD
//    //            }
//    //
//    //            // ------------------------Continue------------------------
//    //            // Add 90 degrees to change AprilTag heading information to same orientation as robot.
//    ////            poseWorldTurret = new Pose(poseWorldTurret.getX(), poseWorldTurret.getY(),poseWorldTurret.getHeading() + Math.PI/2);
//    //
//    //            // Transforming the turret pose into the robot (FIELD RELATIVE) pose--> Pose multiplication: pWR = pWT * pRT^-1.
//    //            // The calculation above is now unnecessary because the camera is mounted on the robot, not the turret
//    //            Pose poseWorldRobot = poseWorldTurret/*.(Robot.Constants.cameraPos.)*/;
//    //            txWorldPinpoint = poseWorldRobot;
//    //            // Set the localizer pose to the current field-relative pose based on AprilTag reading.
//    //            base.setPose(FTCToPedro(poseWorldRobot));//.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
//    //            Positions.flywheelPowerOffset = 0;
//    //            Positions.turretAngleManualOffset = 0;
//    //            // Telemetry displays robot position and heading information
//    //            RobotLog.a("counter ", counter); // display mt1 updates
//    //            RobotLog.a(String.format("Pose XY %6.1f %6.1f  (inch)",
//    //                    poseWorldRobot.getX(),
//    //                    poseWorldRobot.getY()));
//    //            RobotLog.a(String.format("Pose Heading %6.1f  (rad)",
//    //                    poseWorldRobot.getHeading()
//    //            ));
//    //            RobotLog.a(String.format("Pedro Pose XY %6.1f %6.1f  (inch)",
//    //                    FTCToPedro(poseWorldRobot).getX(),
//    //                    FTCToPedro(poseWorldRobot).getY()));
//    //            RobotLog.a(String.format("Pedro Pose Heading %6.1f  (rad)",
//    //                    FTCToPedro(poseWorldRobot).getHeading()
//    //            ));
//    //
//    //            return true; // Returns true for successful relocalization
//        }
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
                if (!Robot.Constants.MINIMIZE_TELEMETRY) {
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
            if (!Robot.Constants.MINIMIZE_TELEMETRY) {
                RobotLog.a("Relocalized using " + filtered.size() + " samples");
            }
            return true;
        }
        //        // ---------------------------Draw Robot Method---------------------------------------
//        public void drawPose(@NonNull TelemetryPacket p){
//            p.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(p.fieldOverlay(), PoseToPose2d(txWorldPinpoint));
//            p.fieldOverlay().setStroke("#00FF00");
//            Drawing.drawRobot(p.fieldOverlay(),PoseToPose2d(txWorldPinpoint).times(Robot.Constants.turretPos));
//            p.fieldOverlay().setStroke("#0000FF");
//            Drawing.drawRobot(p.fieldOverlay(),new Pose2d(Robot.Positions.goalPos.getXComponent(), Robot.Positions.goalPos.getYComponent(), 0));
//        }

    } // END OF APRILDRIVE CLASS
}