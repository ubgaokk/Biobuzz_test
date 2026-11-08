package org.firstinspires.ftc.teamcode.autons; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class AutoRedClose_Teammate_NEW extends OpMode {
    private Robot robot;
    private Robot.AprilFollower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    double lastTime;
    private Pose shootPose = PedroToFTC(new Pose(57+2*(72-57), 83-5, Math.toRadians(-15)));

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
        PICKUPGATE1,
        PICKUPGATE1TURN,
        INTAKINGGATE1,
        SCOREGATE1,
        SCORINGGATE1,
        PICKUPGATE2,
        PICKUPGATE2TURN,
        INTAKINGGATE2,
        SCOREGATE2,
        SCORINGGATE2,
        PICKUPGATE3,
        PICKUPGATE3TURN,
        INTAKINGGATE3,
        SCOREGATE3,
        SCORINGGATE3,
        PICKUP3,
        SCORE3,
        SCORING3,
        PARK,
        DONE
    }
    private PathState pathState;
    private int intakeState;
    private boolean intake;
    private boolean shoot;
    private boolean lock =true;
    private final Pose startPose = new Pose(125.3095 /*121.8235*//*101.6703*/ /*118.83492439024394*/, 112.6529 /*109.0882*//*131.4151*/ /*129.46915609756095*/, 0 /*Math.toRadians(36)*/); // Start Pose of our robot.
    private final Pose scorePose = new Pose(57+2*(72-57), 83-5, Math.toRadians(-15)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(55+2*(72-55), 85-5, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(25+2*(72-25), 85, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(22+2*(72-22), 58-5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(23.5+2*(72-23.5), 53-5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupGatePose = new Pose(23+2*(72-23), 55.25-5, Math.toRadians(35)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupGatePose2 = new Pose(12.5+2*(72-12.5), 64.25-5, Math.toRadians(35)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //    private final Pose intakeGatePose = new Pose(15+2*(72-15), 56, Math.toRadians(60));
    private final Pose parkPose = new Pose(61+2*(72-61), 102, Math.toRadians(-47));

    private Path scorePreload;
    private PathChain grabPickup1,  grabPickup2,  grabPickup3, grabGate1, grabGate1Turn, grabGate2, grabGate2Turn, grabGate3, grabGate3Turn;
    private Path scorePickup1, scorePickup2, scorePickup3, scoreGate1,  scoreGate2, scoreGate3, park;

    boolean reset = true;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(startPose, new Pose(144-50,109.0882,0), scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), 0.75);

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60+2*(72-60),85-5,Math.toRadians(0)), pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = new Path(new BezierLine(pickup1Pose, scorePose2));
        scorePickup1.setLinearHeadingInterpolation(pickup1Pose.getHeading(),scorePose2.getHeading());

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60+2*(72-60),58-5,Math.toRadians(0)), pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = new Path(new BezierLine(pickup2Pose, scorePose));
        scorePickup2.setLinearHeadingInterpolation(pickup2Pose.getHeading(),scorePose.getHeading());

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, new Pose(70+2*(72-70),53-5,Math.toRadians(0)), pickup3Pose))
                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = new Path(new BezierLine(pickup3Pose, parkPose));
        scorePickup3.setLinearHeadingInterpolation(pickup3Pose.getHeading(),parkPose.getHeading(),0.1);
        grabGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40+2*(72-40),48-5,0), pickupGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupGatePose.getHeading(),0.5)
                .build();
        grabGate1Turn = follower.pathBuilder()
                .addPath(new BezierLine(pickupGatePose, pickupGatePose2))
                .setLinearHeadingInterpolation(pickupGatePose.getHeading(),pickupGatePose2.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate1 = new Path(new BezierCurve(pickupGatePose, new Pose(40+2*(72-40),60-5,0), scorePose));
        scoreGate1.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading(),0.2);
        grabGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40+2*(72-40),48-5,0), pickupGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupGatePose.getHeading(),0.5)
                .build();
        grabGate2Turn = follower.pathBuilder()
                .addPath(new BezierLine(pickupGatePose, pickupGatePose2))
                .setLinearHeadingInterpolation(pickupGatePose.getHeading(),pickupGatePose2.getHeading())
                .build();
        //        grabGate1.setBrakingStart(2);

//        intakeGate2 = new Path(new BezierLine(pickupGatePose,intakeGatePose));
//        intakeGate2.setLinearHeadingInterpolation(pickupGatePose.getHeading(), intakeGatePose.getHeading());

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate2 = new Path(new BezierCurve(pickupGatePose, new Pose(40+2*(72-40),60-5,0), scorePose));
        scoreGate2.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading(),0.2);

        grabGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40+2*(72-40),48-5,0), pickupGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupGatePose.getHeading(),0.5)
                .build();
        grabGate3Turn = follower.pathBuilder()
                .addPath(new BezierLine(pickupGatePose, pickupGatePose2))
                .setLinearHeadingInterpolation(pickupGatePose.getHeading(),pickupGatePose2.getHeading())
                .build();
        //        grabGate1.setBrakingStart(2);

//        intakeGate2 = new Path(new BezierLine(pickupGatePose,intakeGatePose));
//        intakeGate2.setLinearHeadingInterpolation(pickupGatePose.getHeading(), intakeGatePose.getHeading());

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate3 = new Path(new BezierCurve(pickupGatePose, new Pose(40+2*(72-40),60-5,0), scorePose));
        scoreGate3.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading(),0.2);

        park = new Path(new BezierLine(scorePose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.getConstraints().setBrakingStart(2);
                follower.getConstraints().setBrakingStrength(0.6);
                follower.followPath(scorePreload);
                intake = true;
                shoot = false;
                lock = true;
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
                if (follower.getDistanceRemaining() < 5){
                    lock = false;
                    intake = false;
                }
                if (!follower.isBusy()) {
                    /* Score Preload */
                    setPathState(PathState.SCORINGPRELOAD); //every time set path state is called, pathtimer is reset
                    RobotLog.a("score preload -> scoring preload");
                }
                break;
            case SCORINGPRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = false;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStart(1);
                    follower.getConstraints().setBrakingStrength(1);
                    follower.followPath(grabPickup2, false);
                    setPathState(PathState.PICKUP2);
                    RobotLog.a("scoring preload -> pickup 1");
                }
                break;
            case PICKUP2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Move to Score Balls */
                    follower.getConstraints().setBrakingStrength(0.75);
//                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(scorePickup2, true);
                    setPathState(PathState.SCORE2);
                }
                break;
            case SCORE2:
                if (follower.getDistanceRemaining() < 2){
//                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Balls */
                    setPathState(PathState.SCORING2);
                }
                break;
            case SCORING2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
//                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.65);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(grabGate1, true);
                    setPathState(PathState.PICKUPGATE1);
                }
                break;
            case PICKUPGATE1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (/*!follower.isBusy() && */pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Move to Score Balls */
                    follower.followPath(grabGate1Turn, true);
                    setPathState(PathState.PICKUPGATE1TURN);
                }
                break;
            case PICKUPGATE1TURN:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    /* Move to Score Balls */
                    setPathState(PathState.INTAKINGGATE1);
                    follower.startTeleOpDrive(true);
                }
                break;
            case INTAKINGGATE1:
                follower.setTeleOpDrive(0.1,0,-0.07,true);
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(0.75);
//                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(scoreGate1);
                    setPathState(PathState.SCOREGATE1);
                }
                break;
            case SCOREGATE1:
                if (follower.getDistanceRemaining() < 2){
//                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    setPathState(PathState.SCORINGGATE1);
                }
                break;
            case SCORINGGATE1:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
//                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.65);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(grabGate2, true);
                    setPathState(PathState.PICKUPGATE2);
                }
                break;
            case PICKUPGATE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (/*!follower.isBusy() && */pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Move to Score Balls */
                    follower.followPath(grabGate2Turn, true);
                    setPathState(PathState.PICKUPGATE2TURN);
                }
                break;
            case PICKUPGATE2TURN:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    /* Move to Score Balls */
                    follower.startTeleOpDrive(true);
                    setPathState(PathState.INTAKINGGATE2);
                }
                break;
            case INTAKINGGATE2:
                follower.setTeleOpDrive(0.1,0,-0.07,true);
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(0.75);
//                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(scoreGate2);
                    setPathState(PathState.SCOREGATE2);
                }
                break;
            case SCOREGATE2:
                if (follower.getDistanceRemaining() < 2){
//                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    setPathState(PathState.SCORINGGATE2);
                }
                break;
            case SCORINGGATE2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
//                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.65);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(grabGate3, true);
                    setPathState(PathState.PICKUPGATE3);
                }
                break;
            case PICKUPGATE3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (/*!follower.isBusy() && */pathTimer.getElapsedTimeSeconds() > 1.5) {
                    /* Move to Score Balls */
                    follower.followPath(grabGate3Turn, true);
                    setPathState(PathState.PICKUPGATE3TURN);
                }
                break;
            case PICKUPGATE3TURN:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    /* Move to Score Balls */
                    follower.startTeleOpDrive(true);
                    setPathState(PathState.INTAKINGGATE3);
                }
                break;
            case INTAKINGGATE3:
                follower.setTeleOpDrive(0.1,0,-0.07,true);
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(0.75);
//                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(scoreGate3);
                    setPathState(PathState.SCOREGATE3);
                }
                break;
            case SCOREGATE3:
                if (follower.getDistanceRemaining() < 2){
//                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    setPathState(PathState.SCORINGGATE3);
                }
                break;
            case SCORINGGATE3:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
//                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(1);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(grabPickup1, false);
                    setPathState(PathState.PICKUP1);
                }
                break;
            case PICKUP1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    shootPose = PedroToFTC(scorePose2);
                    follower.getConstraints().setBrakingStrength(1.0);
                    follower.followPath(scorePickup1, true);
                    setPathState(PathState.SCORE1);
                }
                break;
            case SCORE1:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
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
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
//                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.7);
                    follower.followPath(grabPickup3, false);
                    setPathState(PathState.PICKUP3);
                }
                break;
            case PICKUP3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    follower.getConstraints().setBrakingStrength(0.85);
//                    follower.getConstraints().setBrakingStart(2);
//                    follower.followPath(scorePickup3, true);
                    shootPose = PedroToFTC(new Pose(61+2*(72-61), 102, Math.toRadians(-40)));
                    setPathState(PathState.PARK);
                }
                break;
            case SCORE3:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    setPathState(PathState.SCORING3);
                }
                break;
            case SCORING3:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.53){
                    shoot = false;
//                    follower.followPath(park, true);
                    setPathState(PathState.PARK);
                }
                break;
            case PARK:
//                if (!follower.isBusy()){
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(PathState.DONE);
//                }
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
        robot.updateShooter2(telemetry, true, false, false, false, false, false, shootPose, 0,false,false,false,false);
        autonomousPathUpdate();
        //autonomousSubsystemsUpdate();
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            double curTime = pathTimer.getElapsedTime();
            telemetry.addData("loop time (ms)", curTime - lastTime);
            lastTime = curTime;
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
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
        RobotLog.a("f!!!");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        telemetry.setMsTransmissionInterval(200); //can change later
        robot = Robot.startInstance(startPose, Robot.Color.RED);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO);
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
        robot.startAuto();
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