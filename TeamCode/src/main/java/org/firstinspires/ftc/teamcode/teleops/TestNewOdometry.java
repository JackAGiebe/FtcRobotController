package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomationizing.NewWorldPosition;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@TeleOp(name = "TestOdo", group = "test")
public class TestNewOdometry extends LinearOpMode {

    Constants constants = new Constants();
    MathFunctions mathFunctions = new MathFunctions();
    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this);
    NewWorldPosition newWorldPosition = new NewWorldPosition(0, 0, 0, robotHardware, this);

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, false, true);

        while(!isStarted()){
            newWorldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            newWorldPosition.updateWorldPosition();
        }

        while (!isStopRequested() && opModeIsActive()){
            newWorldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            newWorldPosition.updateWorldPosition();
            teleOpControls.driveControls();

            telemetry.addData("X", newWorldPosition.getxPosition());
            telemetry.addData("Y", newWorldPosition.getyPosition());
            telemetry.addData("Angle", newWorldPosition.getAngleDegrees());
            telemetry.addData("Left Position", robotHardware.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Position", robotHardware.rightEncoder.getCurrentPosition());
            telemetry.addData("Left", newWorldPosition.getLeft());
            telemetry.addData("Right", newWorldPosition.getRight());
            telemetry.addData("Normal Position", robotHardware.normalEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
