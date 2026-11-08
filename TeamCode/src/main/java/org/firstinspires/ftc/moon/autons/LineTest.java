package org.firstinspires.ftc.moon.autons; // make sure this aligns with class location


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.moon.drive.MoonRobot;

//@Disabled
@Autonomous
public class LineTest extends OpMode {
    public Pose PedroToFTC(Pose pose){
        Pose normalizedPose = pose.minus(new Pose(72, 72));
        return normalizedPose.rotate(Math.PI / 2, true);
    }
    MoonRobot robot;
    public MoonRobot.AprilFollower follower;
    // Inches to drive forward. Heading 0 faces +X in Pedro coordinates, so forward = +X.
    public static double DISTANCE = 4;
    private Pose shootPose = PedroToFTC(new Pose(130, 130, Math.toRadians(45)));

    private Path forwards;

    @Override
    public void init() {
        robot = MoonRobot.createInstance(new Pose(72, 72), MoonRobot.Color.RED);
        robot.initAuto(hardwareMap, telemetry, MoonRobot.OpModeState.AUTO);
//        robot.stopper.close();
        follower = robot.follower;
//        follower.setStartingPose(MoonRobot.FTCToPedro(new Pose(0, 0, Math.PI / 2 )));
//        follower.setStartingPose(new Pose(72, 72,Math.PI / 2));
        follower.setPose(new Pose(72, 72, 0 ));

    }

    /** This initializes the Follower before the forward Path is created in start(). */
    @Override
    public void init_loop() {
        telemetry.addLine("The robot will drive forward " + DISTANCE + " inches once, then stop.");
        Pose currentPose = robot.getPose();
        telemetry.addLine("Driving forward: " + follower.isBusy());
        telemetry.addData("Pose X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Pose Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Pose H", String.format("%.1f deg", Math.toDegrees(currentPose.getHeading())));
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(72 + DISTANCE, 72)));
        forwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);

    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        robot.clearBulkCache();
        follower.update();

//        robot.updateIntake(true, false, false, false, telemetry);
//        robot.updateShooter(telemetry, true, false, false, true, false, false, shootPose, 0,false,false,false,false);
        Pose currentPose = robot.getPose();
        telemetry.addLine("Driving forward: " + follower.isBusy());
        telemetry.addData("Pose X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Pose Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Pose H", String.format("%.1f deg", Math.toDegrees(currentPose.getHeading())));
        Pose end_pose = forwards.endPose();
        telemetry.addData("ePose X", String.format("%.2f", end_pose.getX()));
        telemetry.addData("ePose Y", String.format("%.2f", end_pose.getY()));
        telemetry.addData("ePose H", String.format("%.1f deg", Math.toDegrees(end_pose.getHeading())));
        Pose start_pose = forwards.getPose(0);
        telemetry.addData("sPose X", String.format("%.2f", start_pose.getX()));
        telemetry.addData("sPose Y", String.format("%.2f", start_pose.getY()));
        telemetry.addData("sPose H", String.format("%.1f deg", Math.toDegrees(start_pose.getHeading())));

        telemetry.update();
    }
}