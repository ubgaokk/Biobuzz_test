package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;


public class Shooter {
    private Hood hood;
    private Turret turret;
    private Flywheel flywheel;
    public ShooterSystem.ShooterLookupTable table;

    // CONSTANTS
    final double G = 386.22;
//    final double FLIGHT_TIME = Robot.Constants.; // Ball trajectory time from ground to ground flight time isn't a constant
//    double FLYWHEEL_MIN_VEL = Robot.Constants.FLYWHEEL_MIN_VEL;
//    final double FLYWHEEL_RADIUS = Robot.Constants.FLYWHEEL_RADIUS;
//    final double FLYWHEEL_TICKS_PER_REV = Robot.Constants.FLYWHEEL_TICKS_PER_REV;

    double flywheelVel;
    double hoodAngle;
    double turretAngle;
    Pose pose;
    Vector goalVector;
    double goalVectorAngle;
    double distance;

    public Shooter (HardwareMap hardwareMap){
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        table = ShooterSystem.createDefaultTable();
    }
//    // METHODS
//    public Vector getGoalVector(Pose robotPose) {
//        Pose turretPose = robotPose.plus(Robot.Constants.turretPos);
//        return Robot.Positions.goalPos.minus(turretPose.getAsVector());
//    }
    

    // Adjust the robot Pose based on its current movement
    public Pose adjustMovementPose(Pose robotPose, Vector linearVel, double angularVel) {
        double p = 0.65; // Fraction of time along trajectory from ground to ground
        double g = 386.22; // Gravity (in/s^2)
        double flightTime = Math.sqrt(2 * Robot.Positions.deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground
        double newHeading = robotPose.getHeading() + angularVel * flightTime; //actually ang velocity shouldn't affect the estimated shot because the turret is near the center enough
        Pose correctedPose = new Pose(
                robotPose.getX() + linearVel.getXComponent() * flightTime,
                robotPose.getY() + linearVel.getYComponent() * flightTime,
                robotPose.getHeading() /*newHeading*/);
//        Pose correctedPose = new Pose(robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getXComponent(),robotPose.getAsVector().plus(linearVel.times(FLIGHT_TIME)).getYComponent(),robotPose.getHeadingAsUnitVector().times(angularVel * FLIGHT_TIME).getMagnitude());
        return correctedPose;
    }

    public boolean isAimed(){
        return Math.abs(flywheel.getVel() - flywheelVel) < 50 && Math.abs(turret.getTurretRobotAngle() - turretAngle) < 0.1; //incl servo angles later IF we use axon
    }

    //shooting that allows human adjustment and human feeding
    public void update(boolean shoot, boolean humanFeed, Pose robotPose, boolean flywheelUp, boolean flywheelDown, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight) {
        // Adjustment values, static so they can be easily accessed
        if(flywheelUp) {Robot.Positions.flywheelPowerOffset += 0.001;}
        if(flywheelDown) {Robot.Positions.flywheelPowerOffset -= 0.001;}
        if(hoodUp) {Robot.Positions.hoodAngleManualOffset += Math.toRadians(1);}
        if(hoodDown) {Robot.Positions.hoodAngleManualOffset -= Math.toRadians(1);}
        if(turretLeft) {Robot.Positions.turretAngleManualOffset += Math.toRadians(1);}
        if(turretRight) {Robot.Positions.turretAngleManualOffset -= Math.toRadians(1);}

        // ------------------------Shooting trajectory values-----------------------
        double p = Robot.Positions.p; //Robot.Positions.p; // Fraction of time along trajectory from ground to ground
        double g = G; // Gravity (in/s^2)
        double deltaH = Robot.Positions.deltaH; // Height difference from shooter to goal
        double flightTime = Math.sqrt(2 * deltaH / (p * g * (1 - p))); // Ball trajectory time from ground to ground

        Pose2d pose2d = new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        // Calculate shot vector using TURRET's position on the robot and ROBOT's heading---
        Pose2d poseTurret = pose2d.times(Robot.Constants.turretPos);
        pose = new Pose(poseTurret.position.x, poseTurret.position.y, poseTurret.heading.toDouble()); // Turret's field-relative position
        goalVector = Robot.Positions.goalPos.minus(pose.getAsVector());
        distance = goalVector.getMagnitude(); // Horizontal distance
        goalVectorAngle = goalVector.getTheta(); // Angle

        // -----Calculate hood angle, flywheel velocity, and turret angle--------
        double theta = Math.atan(deltaH / (distance * (1 - p))); // Ball launch angle of elevation
        double vel = distance / (p * flightTime * Math.cos(theta)); // Ball launch speed
        double heading = pose.getHeading(); //
        turretAngle = goalVectorAngle - heading; // Angle to turn turret to (relative to robot's heading)
        double wheelRadius = 1.89;

        // Convert vel (speed) to rad/s (example calibration: vel = wheelRadius * rad/s
        double radps = vel / wheelRadius; // RPM

        ShooterSystem.ShooterPoint point = table.get(distance, 14);
        flywheelVel = radps * 28 / Math.PI / 2 * (Robot.Positions.flywheelPower + Robot.Positions.flywheelPowerOffset);
//        flywheelVel = point.rpm;
        hoodAngle = theta+ Robot.Constants.hoodAngleOffset+ Robot.Positions.hoodAngleManualOffset;
//        hoodAngle = point.angle;
        turretAngle = turretAngle+ Robot.Constants.turretAngleOffset+ Robot.Positions.turretAngleManualOffset;

        if (!shoot){
            flywheel.setPower(0);
//            turret.turnToRobotAngle(Math.toDegrees(turretAngle)); //TODO: delete
            //don't move hood and turret
        }
        else if (humanFeed) {
            flywheel.spinTo(Robot.Constants.humanFeedVel);
            hood.turnToAngle(Math.PI/2);
            turret.turnToRobotAngle(0);
        }
        else {
            flywheel.spinTo(flywheelVel);
            hood.turnToAngle(hoodAngle);
            turret.turnToRobotAngle(turretAngle);
//            RobotLog.a(""+Math.toDegrees(turretAngle));
        }
//        telemetry.update();
    }
    public double flightTimeFromDistance(double d) {
        // Replace this with a regression later
        if (d > 80) return 26.0/30;
        return 21.0/30;
    }

    public void update2(
            boolean shoot, boolean humanFeed, boolean moveShot, boolean rotateShot, Pose robotPose, Vector robotVel, Vector robotAccel, double angVel, boolean flywheelUp, boolean flywheelDown, boolean hoodUp, boolean hoodDown, boolean turretLeft, boolean turretRight
    ) {
        // Adjustment values, static so they can be easily accessed
        if(flywheelUp) {Robot.Positions.flywheelPowerOffset += 0.001;}
        if(flywheelDown) {Robot.Positions.flywheelPowerOffset -= 0.001;}
        if(hoodUp) {Robot.Positions.hoodAngleManualOffset += Math.toRadians(0.1);}
        if(hoodDown) {Robot.Positions.hoodAngleManualOffset -= Math.toRadians(0.1);}
        if(turretLeft) {Robot.Positions.turretAngleManualOffset += Math.toRadians(1);}
        if(turretRight) {Robot.Positions.turretAngleManualOffset -= Math.toRadians(1);}

        /* ---------------- Turret pose (geometry only) ---------------- */
        Pose2d pose2d = new Pose2d(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        Pose2d turretPose = pose2d.times(Robot.Constants.turretPos);
        pose = new Pose(turretPose.position.x, turretPose.position.y, turretPose.heading.toDouble());
        Vector turretPos = pose.getAsVector();
        /* ---------------- Turret velocity (dynamics) ---------------- */

// turret offset in robot frame → rotate to field frame
        Vector r = new Vector();
        r.setOrthogonalComponents(Robot.Constants.turretPos
                .position.x,Robot.Constants.turretPos
                .position.y);
        r.rotateVector(robotPose.getHeading());

        // angular velocity contribution: ω × r
        Vector vAngular = new Vector(
                -angVel * r.getYComponent(),
                angVel * r.getXComponent()
        );
//        Vector centripetal = r.times(-angVel * angVel);
//        Vector turretAccel =
//                robotAccel.plus(centripetal);


        Vector turretVel = robotVel.plus(vAngular);
        /* ---------------- Distance + flight time ---------------- */

        Vector goalVec = Robot.Positions.goalPos.minus(turretPos);

        double flightTime = flightTimeFromDistance(distance);
        /* ---------------- Predict turret position at impact ---------------- */

        Vector predictedTurretPos = turretPos.plus(turretVel.times(flightTime))/*.plus(turretAccel.times(0.5 * flightTime * flightTime))*/;
        /* ---------------- Final aiming vector ---------------- */

        goalVector = Robot.Positions.goalPos.minus(predictedTurretPos);
        if (!moveShot) goalVector = goalVec;
        Vector goalDir = goalVector.normalize();

        turretAngle = goalVector.getTheta() - robotPose.getHeading();

//        if (robotPose.getX() > 15) {
//            Robot.Constants.turretAngleOffset = Math.toRadians(4.5) * -turretAngle *15/Math.PI;
////            Robot.Positions.flywheelPower = Robot.Constants.flywheelPower_Far + Math.abs(turretAngle/100.);
//        }
//        else{
//            Robot.Constants.turretAngleOffset = 0;
////            Robot.Positions.flywheelPower = Robot.Constants.flywheelPower;
//        }

        distance = goalVector.getMagnitude();

        /* ---------------- Shooter physics (still valid) ---------------- */

        double deltaH = Robot.Positions.deltaH;
        double p = Robot.Positions.p;
        double g = G;
        double theta = Math.atan(deltaH / (distance * (1 - p)));

        double t = Math.sqrt(2 * deltaH / (p * g * (1 - p)));

        double launchVel = distance / (p * t * Math.cos(theta));

        double wheelRadius = 1.89;
        double flywheelRadPerSec = launchVel / wheelRadius;

        double radialSpeed = -turretVel.dot(goalDir);

        /* ---------------- Apply offsets ---------------- */

        turretAngle += Robot.Constants.turretAngleOffset
                + Robot.Positions.turretAngleManualOffset;

        hoodAngle = theta + Robot.Constants.hoodAngleOffset
                        + Robot.Positions.hoodAngleManualOffset;

        flywheelVel = flywheelRadPerSec * 28 / (2 * Math.PI)
                        * (Robot.Positions.flywheelPower
                        + Robot.Positions.flywheelPowerOffset);

        /* ---------------- Control outputs ---------------- */

        if (!shoot) {
            flywheel.setPower(0);
            return;
        }

        if (humanFeed) {
            flywheel.spinTo(Robot.Constants.humanFeedVel);
            hood.turnToAngle(Math.PI / 2);
            turret.turnToRobotAngle(0);
            return;
        }

        // feedforward: cancel robot rotation
        double turretFF = -angVel * Robot.Constants.TURRET_FF_TIME;
        if (!rotateShot) turretFF = 0;
        if (moveShot){flywheel.setRadialFeedforward(radialSpeed);}
        else{flywheel.setRadialFeedforward(0);}

        turret.turnToRobotAngle(turretAngle + turretFF);
        hood.turnToAngle(hoodAngle);
        flywheel.spinTo(flywheelVel);
    }
    public void sendTelemetry(Telemetry telemetry){
        // ---------------------------TELEMETRY LINES--------------------------------
        telemetry.addData("flywheel power scale factor", Robot.Positions.flywheelPower + Robot.Positions.flywheelPowerOffset);
        telemetry.addData("turret angle offset manual(rad to deg)", Robot.Positions.turretAngleManualOffset*180/Math.PI);
        telemetry.addData("hood angle offset manual (rad to deg)", Robot.Positions.hoodAngleManualOffset*180/Math.PI);
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            if (hood.HOOD.commandedOutsideRange()) telemetry.addLine("WARNING: hood commanded out of its range! Auto set to 0 or 1.");
//        if (turret.turret.commandedOutsideRange()) telemetry.addLine("WARNING: turret commanded out of its range! Auto set to 0 or 1.");
            telemetry.addData("p", Robot.Positions.p);
            telemetry.addData("h", Robot.Positions.deltaH);
    //            telemetry.addData("human feed", humanFeed); //NOT IMPORTANT
    //            telemetry.addData("setPose", toggleLock); //NOT IMPORTANT
            telemetry.addLine("robotTurretPose (inchxinchxdeg): " + pose.getX()+" "+pose.getY()+" "+pose.getHeading()); //NOT IMPORTANT
            telemetry.addLine("goalVector (inchxinch): " + goalVector.getXComponent()+" "+goalVector.getYComponent());
            telemetry.addLine("goalPos (inchxinch): " + Robot.Positions.goalPos.getXComponent()+" "+ Robot.Positions.goalPos.getYComponent()); //NOT IMPORTANT
            // telemetry.addData("goalVector angle (rad to deg)", Math.toDegrees(goalVectorAngle)); //NOT IMPORTANT
            telemetry.addData("distance to goal (inch)", distance); //NOT IMPORTANT
            telemetry.addData("turret angle (rad to deg)", turretAngle*180/Math.PI);
            telemetry.addData("turret angle offset (rad to deg)", Robot.Constants.turretAngleOffset*180/Math.PI);
            telemetry.addData("turret get angle (rad to deg)", turret.getTurretRobotAngle()); //NOT IMPORTANT
            telemetry.addData("turret angle", turret.getTurretRobotAngle());
    //        telemetry.addData("turret current rotation", turret.turret.getCurrentAngle());
    //        telemetry.addData("turret total rotation", turret.turret.getTotalRotation());
            telemetry.addData("turret pos", turret.turret.getPosition()); //NOT IMPORTANT
            telemetry.addData("hood theta (rad to deg)", hoodAngle*180/Math.PI);
            telemetry.addData("hood angle offset (rad to deg)", Robot.Constants.hoodAngleOffset*180/Math.PI);
            telemetry.addData("hood get angle (rad to deg)", hood.getAngle()); //NOT IMPORTANT
            telemetry.addData("hood pos", hood.HOOD.getPosition()); //NOT IMPORTANT
            telemetry.addData("targetVel (rad/s to tick/s)", flywheelVel);
    //            telemetry.addData("targetVel (rad/s)", radps); //NOT IMPORTANT
            telemetry.addData("motorSpeed (tick/s)", flywheel.getVel());
            telemetry.addData("motorSpeed (tick/s to rad/s)", flywheel.getVel() * 2 * Math.PI / 28); //NOT IMPORTANT
        }
    }

    public Hood getHood() {
        return hood;
    }

    public Turret getTurret() {
        return turret;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }
}
