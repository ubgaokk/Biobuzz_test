package org.firstinspires.ftc.teamcode.autons; // make sure this aligns with class location


import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class Line extends OpMode {
    public Pose PedroToFTC(Pose pose){
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(Math.PI / 2, true);
    }
    Robot robot;
    public Robot.AprilFollower follower;
    public static double DISTANCE = 4;
    private boolean forward = true;
    private Pose shootPose = PedroToFTC(new Pose(130, 130, Math.toRadians(45)));

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        robot = Robot.startInstance(new Pose(72, 72), Robot.Color.RED);
        robot.initAuto(hardwareMap, telemetry, Robot.OpModeState.AUTO);
        robot.stopper.close();
        follower = robot.follower;
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetry.addLine("This will activate all the PIDF(s)");
        telemetry.addLine("The robot will go forward and backward continuously along the path while correcting.");
        telemetry.addLine("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        robot.clearBulkCache();
        follower.update();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        robot.updateIntake(true, false, false, false, telemetry);
        robot.updateShooter(telemetry, true, false, false, true, false, false, shootPose, 0,false,false,false,false);
        telemetry.addLine("Driving Forward?: " + forward);
        telemetry.update();
    }
}