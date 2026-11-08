package org.firstinspires.ftc.teamcode.autons; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class AutoBlueFar_Vision extends OpMode {
    private Robot robot;
    private Robot.AprilFollower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean tunnelReady = false;
    double y = 6.63;
    private enum PathState{
        START,
        SCOREPRELOAD,
        SCORINGPRELOAD,
        PICKUP1,
        SCORE1,
        SCORING1,
        PICKUP2,
        SCORE2,
        SCORING2,
        PICKUP3,
        SCORE3,
        SCORING3,
        PICKUP4,
        SCORE4,
        SCORING4,
        PICKUP5,
        SCORE5,
        SCORING5,
        PARK,
        DONE
    }
    private PathState pathState;
    private int intakeState;
    private boolean intake = false;
    private boolean shoot = false;
    private final Pose startPose = new Pose(54.7899 /*144-88.4213*/, 8.15/*8.576*//*6.6302*/, Math.toRadians(90/*180*/)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(144-88.4213, 17.6302, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose shootPose = PedroToFTC(scorePose);
    private final Pose pickup1Pose = new Pose(144-132.5624, 6.6316, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(20, 40, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = pickup1Pose; // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup4Pose = pickup1Pose; // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup5Pose = pickup1Pose; // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup45Pose = new Pose(144-132.5624, 16.6316, Math.toRadians(180));
    private final Pose pickup45Pose_rest = new Pose(144-125.5624, 9.6316, Math.toRadians(180));

    private Pose gatePose = new Pose(144-126.5624, 35, Math.toRadians(180)); // tunnel clustered
    private final Pose loadingPose = new Pose(144-136, 9, Math.toRadians(180));

    //    private final Pose intakeGatePose = new Pose(15+2*(72-15), 56, Math.toRadians(60));
    private final Pose parkPose = new Pose(144-98.4213, 17.6302, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1,  grabPickup2,  grabPickup3, grabPickup4, grabPickup5, grabPickup5_2, grabPickup5_3, tunnelPickup, tunnelSweep;
    private Path scorePickup1, scorePickup2, scorePickup3, scorePickup4,  scorePickup5, tunnelScore, sweepScore, park;

    boolean reset = true;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = new Path(new BezierLine(pickup1Pose, scorePose));
        scorePickup1.setConstantHeadingInterpolation(scorePose.getHeading());

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(50,40,Math.toRadians(180)), pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        tunnelPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(50, gatePose.getY(), Math.toRadians(180)), gatePose))
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();
        tunnelSweep = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(144-110, 50, Math.toRadians(0)), gatePose))
//                .setConstantHeadingInterpolation(gatePose.getHeading())
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(gatePose, new Pose(144-137, 20, Math.toRadians(0)), loadingPose))
//                .setConstantHeadingInterpolation(loadingPose.getHeading())
                .setTangentHeadingInterpolation()
                .build();
        tunnelScore = new Path(new BezierLine(gatePose,scorePose));
                tunnelScore.setConstantHeadingInterpolation(gatePose.getHeading());
        sweepScore = new Path(new BezierLine(loadingPose,scorePose));
        sweepScore.setConstantHeadingInterpolation(scorePose.getHeading());
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = new Path(new BezierLine(pickup2Pose, scorePose));
        scorePickup2.setLinearHeadingInterpolation(pickup2Pose.getHeading(),scorePose.getHeading());

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
//                .build();
        grabPickup3 = grabPickup2;

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = new Path(new BezierLine(pickup3Pose, scorePose));
//        scorePickup3.setConstantHeadingInterpolation(scorePose.getHeading());
        scorePickup3 = scorePickup2;
//        grabPickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup45Pose))
//                .setConstantHeadingInterpolation(pickup45Pose.getHeading())
//                .build();
        grabPickup4 = grabPickup2;
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup4 = new Path(new BezierLine(pickup45Pose, scorePose));
//        scorePickup4.setConstantHeadingInterpolation(scorePose.getHeading());
        scorePickup4 = scorePickup2;
        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup5Pose))
                .setConstantHeadingInterpolation(pickup5Pose.getHeading())
                .build();
//        grabPickup5_2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup45Pose, pickup45Pose_rest))
//                .setConstantHeadingInterpolation(pickup5Pose.getHeading())
//                .build();
//        grabPickup5_3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup45Pose_rest, pickup5Pose))
//                .setConstantHeadingInterpolation(pickup5Pose.getHeading())
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup5 = new Path(new BezierLine(pickup5Pose, scorePose));
        scorePickup5.setConstantHeadingInterpolation(scorePose.getHeading());

        park = new Path(new BezierLine(scorePose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.getConstraints().setBrakingStart(4);
                follower.getConstraints().setBrakingStrength(0.5);
                follower.followPath(scorePreload);
                intake = true;
                shoot = false;
                setPathState(PathState.SCOREPRELOAD);
                RobotLog.a("start -> score preload");
                break;
            case SCOREPRELOAD:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds() > 2.7){
                    intake = false;
                    /* Score Preload */
                    setPathState(PathState.SCORINGPRELOAD); //every time set path state is called, pathtimer is reset
                    RobotLog.a("score preload -> scoring preload");
                }
                break;
            case SCORINGPRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.0){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.3){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    /* Move to Intake Balls */
                    follower.followPath(grabPickup1, false);
                    setPathState(PathState.PICKUP1);
                    RobotLog.a("scoring preload -> pickup 1");
                }
                break;
            case PICKUP1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Move to Score Balls */
                    follower.getConstraints().setBrakingStrength(0.7);
                    follower.followPath(scorePickup1, true);
                    setPathState(PathState.SCORE1);
                }
                break;
            case SCORE1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Balls */
                    setPathState(PathState.SCORING1);
                }
                break;
            case SCORING1:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(1);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(grabPickup2, false);
                    setPathState(PathState.PICKUP2);
                }
                break;
            case PICKUP2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Move to Score Balls */
                    follower.getConstraints().setBrakingStrength(0.7);
                    follower.getConstraints().setBrakingStart(4);
                    follower.followPath(scorePickup2, true);
                    setPathState(PathState.SCORE2);
                }
                break;
            case SCORE2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Balls */
                    setPathState(PathState.SCORING2);
                }
                break;
            case SCORING2:
                tunnelReady = true;
                y=6.63;
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    tunnelReady = false;
                    /* Move to Intake Balls */
                    follower.followPath(tunnelPickup, true);
                    setPathState(PathState.PICKUP3);
                }
                break;
            case PICKUP3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    /* Grab Sample */
                    /* Move to Score Balls */
                    follower.followPath(tunnelScore, true);
                    setPathState(PathState.SCORE3);
                }
                break;
            case SCORE3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Balls */
                    setPathState(PathState.SCORING3);
                }
                break;
            case SCORING3:
                tunnelReady = true;
                y=6.63;
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    /* Move to Intake Balls */
                    tunnelReady = false;
                    follower.followPath(tunnelPickup, true);
                    setPathState(PathState.PICKUP4);
                }
                break;
            case PICKUP4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()|| pathTimer.getElapsedTimeSeconds() > 2.5) {
                    /* Grab Sample */
                    /* Move to Score Balls */
                    follower.followPath(tunnelScore, true);
                    setPathState(PathState.SCORE4);
                }
                break;
            case SCORE4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Balls */
                    setPathState(PathState.SCORING4);
                }
                break;
            case SCORING4:
                tunnelReady = true;
                y=6.63;
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    /* Move to Intake Balls */
                    tunnelReady = false;
                    follower.followPath(tunnelPickup, true);
                    setPathState(PathState.PICKUP5);
                }
                break;
            case PICKUP5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()|| pathTimer.getElapsedTimeSeconds() > 2.5) {
                    /* Move to Score Balls */
                    follower.followPath(tunnelScore, true);
                    setPathState(PathState.SCORE5);
                }
                break;
//            case PICKUP5_2:
//                if (!follower.isBusy()) {
//                    /* Move to Score Balls */
//                    follower.followPath(grabPickup5_3, true);
//                    setPathState(PathState.PICKUP5_3);
//                }
//                break;
//            case PICKUP5_3:
//                if (!follower.isBusy()) {
//                    /* Move to Score Balls */
//                    follower.followPath(scorePickup5, true);
//                    setPathState(PathState.SCORE5);
//                }
//                break;
            case SCORE5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    setPathState(PathState.SCORING5);
                }
                break;
            case SCORING5:
                tunnelReady = true;
                y=6.63;
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.stopper.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.1){
                    shoot = false;
                    intake = false;
                    tunnelReady = false;
//                    follower.followPath(tunnelPickup, true);
                    setPathState(PathState.PARK);
                }
                break;
            case PARK:
                if (!follower.isBusy()){
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(PathState.DONE);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(PathState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setIntakeState(int iState) {
        intakeState = iState;
        actionTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        robot.clearBulkCache();
        follower.update();
//        robot.updateBallDetector();
        robot.setPose(PedroToFTC(follower.getPose()));
        robot.updateIntake(intake, false, !intake, shoot, telemetry);
        if (tunnelReady/*ready to pickup gate*/) {
            double bestCenter = robot.camera.updateMostBallsX(true, telemetry);
//            double y = 6.63;
            if (!Double.isNaN(bestCenter)){
                y = Math.min(Math.max(Range.scale(bestCenter,39,1160,0,48)-8,6.63),40);
            }
//            Math.max(0 + (48-48),6.63)
            gatePose = new Pose(144-132.5624, y, Math.toRadians(180)); // set target end position
            tunnelPickup = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, new Pose(50, gatePose.getY(), Math.toRadians(180)), gatePose))
                    .setConstantHeadingInterpolation(gatePose.getHeading())
                    .build();
            tunnelScore = new Path(new BezierLine(gatePose,scorePose));
            tunnelScore.setConstantHeadingInterpolation(gatePose.getHeading());
            RobotLog.a("y val " + y);
        }
//        robot.camera.updateMostBallsX(true, telemetry); // find clusters
        robot.updateShooter(telemetry, true, false, false, false, false, false, shootPose, 0,false,false,false,false);
        autonomousPathUpdate();
        // autonomousSubsystemsUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public Pose PedroToFTC(Pose pose){
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(Math.PI / 2, true);
    }
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        RobotLog.a("roe!!!");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        telemetry.setMsTransmissionInterval(200); //can change later
        robot = Robot.startInstance(startPose, Robot.Color.BLUE);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO_FAR);
        robot.camera.switchToBalls();
        robot.stopper.close();
        follower = robot.follower;
        follower.getConstraints().setBrakingStrength(1);
//        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        RobotLog.a("real startPose: " + startPose.getX()+ ","  + startPose.getY()+ ", " + startPose.getHeading());
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PathState.START);
        setIntakeState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}