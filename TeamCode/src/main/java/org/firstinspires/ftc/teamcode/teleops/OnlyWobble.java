package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@TeleOp(name = "Only Wobble", group = "test")
public class OnlyWobble extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware(this);
    Constants constants = new Constants();
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this, worldPosition, robotMovement);
//    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);

    Alliance alliance = Alliance.BLUE;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, false);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            teleOpControls.driveControls();
            teleOpControls.wobbleControls();
        }
    }
}
