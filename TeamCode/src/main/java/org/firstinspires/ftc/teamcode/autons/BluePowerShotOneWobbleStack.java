package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomationizing.AutonFunctions;
import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.Randomization;
import org.firstinspires.ftc.teamcode.enums.drivestates.PowershotOneWobbleStack;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

@Disabled
@Autonomous(name = "Blue Power Shot One Wobble Stack", group = "BNB")
public class BluePowerShotOneWobbleStack extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this);
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    AutonFunctions autonFunctions = new AutonFunctions(robotHardware, robotMovement);

    PowershotOneWobbleStack driveState = PowershotOneWobbleStack.WAIT_FOR_START;
    Randomization randomization = Randomization.A;

    int powerShotsHit = 0;
    int shotsFired = 0;

    ElapsedTime totalTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, true);

        telemetry.addLine("Yeet");
        telemetry.update();

        while(!isStarted()){
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();
//            visionUsage.run();
            randomization = Randomization.A;

//            telemetry.addData("avg 1", visionUsage.getNumber());
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        totalTime.reset();

        autonFunctions.autonStart();

        while(!isStopRequested() && opModeIsActive()){
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();
            robotMovement.setMotorPowers();

            telemetry.addData("Drive State", driveState);
            telemetry.addData("X", worldPosition.getxPosition());
            telemetry.addData("Y", worldPosition.getyPosition());
            telemetry.addData("Angle", worldPosition.getAngleDegrees());
            telemetry.update();

            switch (driveState){
                case WAIT_FOR_START:
                    robotMovement.setDrive(0);
                    robotMovement.setStrafe(0);
                    robotMovement.setRotate(0);
                    autonFunctions.prepShootLowGoal();
                    driveState = PowershotOneWobbleStack.FIRST_POWER_SHOT;
                    break;

                case FIRST_POWER_SHOT:
                    robotMovement.goToPoint(45, 60, 11, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1){
                        powerShotsHit += autonFunctions.shootOnceLow();
                    }
                    if(powerShotsHit == 1)
                        driveState = PowershotOneWobbleStack.SECOND_POWER_SHOT;
                    break;

                case SECOND_POWER_SHOT:
                    robotMovement.goToPoint(45, 59, 7, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1){
                        powerShotsHit += autonFunctions.shootOnceLow();
                    }
                    if(powerShotsHit == 2)
                        driveState = PowershotOneWobbleStack.THIRD_POWER_SHOT;
                    break;

                case THIRD_POWER_SHOT:
                    robotMovement.goToPoint(45, 59, 3, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1){
                        powerShotsHit += autonFunctions.shootOnceLow();
                    }
                    if(powerShotsHit == 3) {
                        if(randomization != Randomization.A) {
                            driveState = PowershotOneWobbleStack.BACK_TO_STACK;
                        }
                        else{
                            driveState = PowershotOneWobbleStack.PLACE_WOBBLE;
                        }
                    }
                    break;

                case BACK_TO_STACK:
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(45, 40, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3)
                        autonFunctions.turnOffShooter();
                    driveState = PowershotOneWobbleStack.INTAKE_STACK;
                    break;

                case INTAKE_STACK:
                    robotMovement.goToPoint(30, 40, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3)

                        driveState = PowershotOneWobbleStack.SHOOT_HIGH;

                case SHOOT_HIGH:
                    robotMovement.goToPoint(50, 59, -7, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5) {
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    if(shotsFired == 3) {
                        if(randomization == Randomization.C)
                            driveState = PowershotOneWobbleStack.BACK_TO_STACK_AGAIN;
                        else
                            driveState = PowershotOneWobbleStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK_AGAIN:
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(45, 40, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 5)
                        autonFunctions.turnOffShooter();
                    driveState = PowershotOneWobbleStack.INTAKE_STACK_AGAIN;
                    break;

                case INTAKE_STACK_AGAIN:
                    robotMovement.goToPoint(22, 40, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = PowershotOneWobbleStack.SHOOT_HIGH_AGAIN;

                case SHOOT_HIGH_AGAIN:
                    robotMovement.goToPoint(50, 59, -7, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5) {
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    if(shotsFired == 3)
                        driveState = PowershotOneWobbleStack.PLACE_WOBBLE;
                    break;

                case PLACE_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(14, 76, -72, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(34, 87, -33, 1, 1);
                    else
                        robotMovement.goToPoint(18, 119, -47, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2){
                        autonFunctions.depositWobble();
                        driveState = PowershotOneWobbleStack.BACK_FROM_WOBBLE;
                    }
                    break;

                case BACK_FROM_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(21, 76, -72, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(37, 82, -41, 1, 1);
                    else
                        robotMovement.goToPoint(25, 111, -56, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2){
                        autonFunctions.liftWobbleArm();
                        driveState = PowershotOneWobbleStack.PARK;
                    }
                    break;

                case PARK:
                    robotMovement.goToPoint(48, 76, 0, 1, 1);
            }
        }
    }
}
