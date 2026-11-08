package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TeleopNew extends OpMode {
//    Turret turret;
    Robot robot;
    boolean intake = false; //false = stop, true = intake
    boolean setPose = false;
    boolean slow = false;
    boolean RT = false;
    boolean LT = false;
    boolean moveShot = false;
    MultipleTelemetry joinedTelemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
    TelemetryPacket p;
    public void init(){
        robot = Robot.getInstance(new Pose(72,72, Math.PI/2), Robot.Color.BLUE); //start facing the goals, BLUE poses
        robot.initTeleop(hardwareMap, telemetry);
        RobotLog.a("logr");
        p = new TelemetryPacket();
        joinedTelemetry.setMsTransmissionInterval(200); //can change later
        joinedTelemetry.update();
    }
    public void start(){
        robot.startTeleop();
    }
    public void loop(){
        //toggles: LB, x, y
        boolean lb; //should be the same as gamepad1.leftBumperWasPressed() i don't know why that's not working
        if (gamepad1.left_bumper && !intake) lb = true;
        else lb = false;
        intake = gamepad1.left_bumper;
        boolean rt = gamepad2.right_trigger > 0.1 && !RT;
        RT = gamepad2.right_trigger > 0.1;
        boolean lt = gamepad2.left_trigger > 0.1 && !LT;
        LT = gamepad2.left_trigger > 0.1;
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) setPose = !setPose;
//        if (gamepad2.yWasPressed()) human = !human;
        if (gamepad2.leftBumperWasPressed() /*|| gamepad1.leftStickButtonWasPressed()*/) slow = !slow;
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) moveShot = !moveShot;

        // UPDATE TELEOP BUTTONS!
//        robot.reportBatteryDraw(joinedTelemetry);
        robot.clearBulkCache();
        robot.updateFollower(gamepad1.startWasPressed() || gamepad2.startWasPressed(), gamepad1.touchpadWasPressed() || gamepad2.backWasPressed(), gamepad2.rightBumperWasPressed(), joinedTelemetry);
        //final version will be: LB = intake toggle, LT = reverse, RB = shoot request, RT = shoot 1
        robot.updateIntake(lb, gamepad1.left_trigger > 0.1, lb, gamepad1.right_bumper, joinedTelemetry);
        robot.setGoalTarget();
        //final version will be: a = spinup request, b = idle request, x = toggle setPose => shooter lock/manual control, rightstick up/down = flywheel scale, dpad up/down= hood, dpad left/right = turret, gamepad2 y = human feed toggle, start = power flywheel off
        robot.updateShooter2(joinedTelemetry,gamepad1.right_bumper, gamepad1.a || gamepad2.a, false, setPose, gamepad1.b || gamepad2.b, moveShot, Robot.Positions.teleShotPose, /*-gamepad1.right_stick_y*/ -gamepad2.right_stick_y, gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed(), gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed(), gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed());
        //final: toggle left stick button = slow mode, toggle y = p2p drive but it stops on its own
        robot.driveTele(y, x, r, slow, false, false, false, joinedTelemetry);
        if (LT) robot.stopper.close();
        else if (RT) robot.stopper.open();
        if (!Robot.Constants.MINIMIZE_TELEMETRY) {
            robot.follower.drawPose(p);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
        joinedTelemetry.update();
    }
}
