package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@TeleOp(name = "Drive while spinning", group = "test")
public class SpinDrive extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this);
    Constants constants = new Constants();
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this, worldPosition, robotMovement);

    boolean forward = true;

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, false);

        telemetry.addLine("Yeet");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();
            robotMovement.setMotorPowers();

            if(forward){
                robotMovement.goToPoint(0, 65, worldPosition.getAngleDegrees() + 20, .5, 1.3);
                if(worldPosition.getyPosition() > 60)
                    forward = false;
            }
            else{
                robotMovement.goToPoint(0, 0, worldPosition.getAngleDegrees() + 10, .5, 1.3);
                if(worldPosition.getyPosition() < 5)
                    forward = true;
            }
            telemetry.update();
        }
    }
}