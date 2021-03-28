package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.TeleOpControls;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@TeleOp(name = "Odo is coolio", group = "test")
public class ShowOff extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this);
    Constants constants = new Constants();
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    TeleOpControls teleOpControls = new TeleOpControls(robotHardware, this, worldPosition, robotMovement);

    public void runOpMode() throws InterruptedException {

        robotHardware.init(hardwareMap, false);

        telemetry.addLine("Yeet");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();
            robotMovement.goToPoint(0, 0, 0, 2, 1);
            robotMovement.setMotorPowers();

//            telemetry.addData("Drive", robotMovement.getDrive());
//            telemetry.addData("Strafe", robotMovement.getStrafe());
//            telemetry.addData("Rotate", robotMovement.getRotate());
            telemetry.addData("X", worldPosition.getxPosition());
            telemetry.addData("Y", worldPosition.getyPosition());
            telemetry.addData("Angle", worldPosition.getAngleDegrees());
//            telemetry.addData("X Power", robotMovement.getxPower());
//            telemetry.addData("Y Power", robotMovement.getyPower());
//            telemetry.addData("Turn Power", robotMovement.getTurnPower());
//            telemetry.addData("Distance to Point", robotMovement.getDistanceToPoint());
//            telemetry.addData("X to Point", robotMovement.getxToPoint());
//            telemetry.addData("Y to Point", robotMovement.getyToPoint());
//            telemetry.addData("Angle to Preffered Magnitude", robotMovement.getAngleToPreferredMagnitude());
//            telemetry.addData("Angle to Preffered Direction", robotMovement.getAngleToPreferredDirection());
//            telemetry.addData("Angle to Preffered", robotMovement.getAngleToPreferred());
//            telemetry.addData("Move Speed", robotMovement.getMoveSpeed());
//            telemetry.addData("Movement X Power", robotMovement.getMovementXPower());
//            telemetry.addData("Movement Y Power", robotMovement.getMovementYPower());
//            telemetry.addData("Movement Turn Power", robotMovement.getMovementTurnPower());
            telemetry.update();
        }
    }
}