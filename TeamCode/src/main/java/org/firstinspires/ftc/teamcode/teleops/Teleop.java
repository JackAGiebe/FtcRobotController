package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.autonomationizing.NewWorldPosition;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.IntakeStatus;
import org.firstinspires.ftc.teamcode.enums.PushStep;
import org.firstinspires.ftc.teamcode.enums.WobblePosition;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this, telemetry);
    Constants constants = new Constants();
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this);
    NewWorldPosition newWorldPosition = new NewWorldPosition(0, 0, 0, robotHardware, this);
//    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);

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

        robotHardware.init(hardwareMap, false, false);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            teleOpControls.driveControls();
            teleOpControls.shooterControls();
            teleOpControls.intakeControls();
            teleOpControls.wobbleControls();
            newWorldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            newWorldPosition.updateWorldPosition();

            telemetry.addData("X", newWorldPosition.getxPosition());
            telemetry.addData("Y", newWorldPosition.getyPosition());
            telemetry.addData("Angle", newWorldPosition.getAngleDegrees());
            telemetry.update();
        }
    }
}