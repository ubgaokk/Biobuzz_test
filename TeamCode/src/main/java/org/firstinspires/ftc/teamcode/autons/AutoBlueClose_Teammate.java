package org.firstinspires.ftc.teamcode.autons; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
@Disabled
@Autonomous
public class AutoBlueClose_Teammate extends OpMode {
    private Robot robot;
    private Robot.AprilFollower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Pose shootPose = PedroToFTC(new Pose(56, 88, Math.toRadians(135)));

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
        INTAKINGGATE1,
        SCOREGATE1,
        SCORINGGATE1,
        PICKUPGATE2,
        INTAKINGGATE2,
        SCOREGATE2,
        SCORINGGATE2,
        PICKUPGATE3,
        INTAKINGGATE3,
        SCOREGATE3,
        SCORINGGATE3,
        PARK,
        DONE
    }
    private PathState pathState;
    private int intakeState;
    private boolean intake;
    private boolean shoot;
    private boolean lock;
    private final Pose startPose = new Pose(144-128.3959, 113.0594+0.8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(56, 89, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(26, 85, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(25, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupGatePose = new Pose(13.5, 61, Math.toRadians(150)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose intakeGatePose = new Pose(15+2*(72-15), 56, Math.toRadians(60));
    private final Pose parkPose = new Pose(52, 105, Math.toRadians(135));

    private Path scorePreload;
    private PathChain grabPickup1,  grabPickup2,  grabGate1, grabGate2, grabGate3;
    private Path scorePickup1, scorePickup2, scorePickup3, scoreGate1,  scoreGate2, scoreGate3, park;

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
                .addPath(new BezierCurve(scorePose, new Pose(60,85,Math.toRadians(180)), pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = new Path(new BezierLine(pickup1Pose, parkPose));
        scorePickup1.setLinearHeadingInterpolation(pickup1Pose.getHeading(),parkPose.getHeading());

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60,58,Math.toRadians(180)), pickup2Pose))
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
        grabGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40,50,Math.toRadians(180)), pickupGatePose))
                .setConstantHeadingInterpolation(pickupGatePose.getHeading())
//                .setBrakingStrength(0.3)
                .build();
//                grabGate1.setBrakingStart(2);
//        intakeGate1 = new Path(new BezierLine(pickupGatePose,intakeGatePose));
//        intakeGate1.setLinearHeadingInterpolation(pickupGatePose.getHeading(), intakeGatePose.getHeading());

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate1 = new Path(new BezierCurve(pickupGatePose, new Pose(40,50,Math.toRadians(180)), scorePose));
        scoreGate1.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading());
        grabGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40,50,Math.toRadians(180)), pickupGatePose))
                .setConstantHeadingInterpolation(pickupGatePose.getHeading())
//                .setBrakingStrength(0.3)
                .build();
        //        grabGate1.setBrakingStart(2);

//        intakeGate2 = new Path(new BezierLine(pickupGatePose,intakeGatePose));
//        intakeGate2.setLinearHeadingInterpolation(pickupGatePose.getHeading(), intakeGatePose.getHeading());

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate2 = new Path(new BezierCurve(pickupGatePose, new Pose(40,50,Math.toRadians(180)), scorePose));
        scoreGate2.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading());

        grabGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40,50,Math.toRadians(180)), pickupGatePose))
                .setConstantHeadingInterpolation(pickupGatePose.getHeading())
//                .setBrakingStrength(0.3)
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreGate3 = new Path(new BezierCurve(pickupGatePose, new Pose(40,50,Math.toRadians(180)), scorePose));
        scoreGate3.setLinearHeadingInterpolation(pickupGatePose.getHeading(),scorePose.getHeading());

        park = new Path(new BezierLine(scorePose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.getConstraints().setBrakingStart(4);
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
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
                }
                if (!follower.isBusy()) {
                    /* Score Preload */
                    intake = false;
                    setPathState(PathState.SCORINGPRELOAD); //every time set path state is called, pathtimer is reset
                    RobotLog.a("score preload -> scoring preload");
                }
                break;
            case SCORINGPRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.77){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStart(1);
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
                    follower.followPath(scorePickup2, true);
                    setPathState(PathState.SCORE2);
                }
                break;
            case SCORE2:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
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
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.67){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.3);
                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(grabGate1, true);
                    setPathState(PathState.PICKUPGATE1);
                }
                break;
            case PICKUPGATE1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    setPathState(PathState.INTAKINGGATE1);
                }
                break;
            case INTAKINGGATE1:
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(1);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(scoreGate1);
                    setPathState(PathState.SCOREGATE1);
                }
                break;
            case SCOREGATE1:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
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
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.67){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.3);
                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(grabGate2, true);
                    setPathState(PathState.PICKUPGATE2);
                }
                break;
            case PICKUPGATE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    setPathState(PathState.INTAKINGGATE2);
                }
                break;
            case INTAKINGGATE2:
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(1);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(scoreGate2);
                    setPathState(PathState.SCOREGATE2);
                }
                break;
            case SCOREGATE2:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
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
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.67){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = true;
                    /* Move to Intake Balls */
                    follower.getConstraints().setBrakingStrength(0.3);
                    follower.getConstraints().setBrakingStart(2);
                    follower.followPath(grabGate3, false);
                    setPathState(PathState.PICKUPGATE3);
                }
                break;
            case PICKUPGATE3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    setPathState(PathState.INTAKINGGATE3);
                }
                break;
            case INTAKINGGATE3:
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                    follower.getConstraints().setBrakingStrength(1);
                    follower.getConstraints().setBrakingStart(1);
                    follower.followPath(scoreGate3);
                    setPathState(PathState.SCOREGATE3);
                }
                break;
            case SCOREGATE3:
                if (follower.getDistanceRemaining() < 2){
                    lock = false;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    intake = false;
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    setPathState(PathState.SCORINGGATE3);
                }
                break;
            case SCORINGGATE3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.67){
                    shoot = false;
                    intake = true;
                    follower.followPath(grabPickup1, true);
                    setPathState(PathState.PICKUP1);
                }
                break;
            case PICKUP1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Move to Score Balls */
                    follower.followPath(scorePickup1, true);
                    shootPose = PedroToFTC(new Pose(52, 107, Math.toRadians(135)));
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
                if (pathTimer.getElapsedTimeSeconds() > 0.67){
                    shoot = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1){
                    shoot = false;
                    intake = true; //maybe problem: stopper gets stuck, could try using analog encoder
                    lock = true;
                    /* Move to Intake Balls */
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
        robot.updateShooter(telemetry, true, false, false, true, false, false, shootPose, 0,false,false,false,false);
        autonomousPathUpdate();
        //autonomousSubsystemsUpdate();
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
        RobotLog.a("glo!!!");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        robot = Robot.startInstance(startPose, Robot.Color.BLUE);
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