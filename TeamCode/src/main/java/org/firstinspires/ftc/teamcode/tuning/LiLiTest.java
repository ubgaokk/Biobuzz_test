package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
//@Disabled

@TeleOp(name = "LiLiApriltagTest", group = "Concept")
public class LiLiTest extends OpMode{

    private Limelight limelight;
//    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init()
    {
        limelight = new Limelight(hardwareMap);
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); // MIGHT CHANGE
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setHeading(90, AngleUnit.DEGREES);
        pinpoint.resetPosAndIMU();
    }

//    @Override
//    public void start()
//    {
//        limelight.start();
//    }

    @Override
    public void loop()
    {
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES) + 180;
//        limelight.updateRobotOrientation(heading);
//        LLResult llResult = limelight.getLatestResult();
//        limelight.updateHeading(heading);
        Pose pose = limelight.update(telemetry);
        if (pose != null) {
//            Pose3D botpose = llResult.getBotpose();
//            Pose3D botpose_mt2 = llResult.getBotpose_MT2();
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//            telemetry.addData("BotPose", botpose.getPosition().toUnit(DistanceUnit.INCH).toString() + "\n" + botpose.getOrientation().toString());
            telemetry.addData("BotPose MT2", pose.toString());
        }
        telemetry.update();
    }
}
