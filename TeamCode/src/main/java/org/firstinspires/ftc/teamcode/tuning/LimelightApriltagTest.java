package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled

@TeleOp(name = "LimelightApriltagTest", group = "Concept")
public class LimelightApriltagTest extends OpMode{

    private Limelight3A limelight;
//    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // limelight 0 pipeline
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); // MIGHT CHANGE
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setHeading(90, AngleUnit.DEGREES);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void start()
    {
        limelight.start();
    }

    @Override
    public void loop()
    {
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES) + 180;
        limelight.updateRobotOrientation(heading);
        LLResult llResult = limelight.getLatestResult();

        if (llResult!=null && llResult.isValid())
        {
            Pose3D botpose = llResult.getBotpose();
            Pose3D botpose_mt2 = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botpose.getPosition().toUnit(DistanceUnit.INCH).toString() + "\n" + botpose.getOrientation().toString());
            telemetry.addData("BotPose MT2", botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).toString() + "\n" + botpose_mt2.getOrientation().toString());
        }
        else {
            telemetry.addData("pinpoint heading", heading);
            telemetry.addLine("NO TAG DETECTED");
        }

        telemetry.update();
    }
}
