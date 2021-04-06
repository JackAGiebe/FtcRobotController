package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@Disabled
@TeleOp(name = "Blue Teleop", group = "TeleOp")
public class BlueTeleop extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this);
    Constants constants = new Constants();
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this, worldPosition, robotMovement);
//    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);

    Alliance alliance = Alliance.BLUE;

    /*
    Drive controls are left stick and triggers.
    Fly wheel is automated.
    Intake toggled with Y (Will automatically turn off when the hopper is up, and back on (intaking) when it drops).
    Hopper toggled with with A.
    Angle up with RB, down with LB.
    Pusher pushes once with X.
    Wobble position toggled with B.
    Wobble secure toggled with right stick button.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, false);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            teleOpControls.driveControls();
            teleOpControls.driveForPowerShots(alliance);
            teleOpControls.shooterControls();
            teleOpControls.intakeControls();
            teleOpControls.wobbleControls();
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();

            telemetry.addData("Angle", teleOpControls.angleUp ? "high" : "low");
            telemetry.addData("X", worldPosition.getxPosition());
            telemetry.addData("Y", worldPosition.getyPosition());
            telemetry.addData("Angle", worldPosition.getAngleDegrees());
            telemetry.update();
        }
    }
}