package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomationizing.AutonFunctions;
import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.Randomization;
import org.firstinspires.ftc.teamcode.enums.drivestates.HighGoalOneWobbleStack;
import org.firstinspires.ftc.teamcode.enums.drivestates.StackJitter;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCVExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Wall High Goal One Wobble Stack", group = "BNB")
public class RedHighGoalOneWobbleStack extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware(this);
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    AutonFunctions autonFunctions = new AutonFunctions(robotHardware, robotMovement);
    Constants constants = new Constants();

    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    HighGoalOneWobbleStack driveState = HighGoalOneWobbleStack.WAIT_FOR_START;
    Randomization randomization = Randomization.A;
    StackJitter stackJitter = StackJitter.FORWARD_ONE;

    int shotsFired = 0;

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime aimTime = new ElapsedTime();
    ElapsedTime postShotTime = new ElapsedTime();
    boolean goodToGo = false;
    boolean aimed = false;
    boolean memeRight = false;

    public void runOpMode() throws InterruptedException {
        robotHardware.init(hardwareMap, true);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "quacktocam"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        while (!isStarted()) {
            worldPosition.giveEncoderHardwareCalls(robotHardware.leftEncoder.getCurrentPosition(), robotHardware.rightEncoder.getCurrentPosition(), robotHardware.normalEncoder.getCurrentPosition());
            worldPosition.updateWorldPosition();
            randomization = pipeline.getRandomization();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Randomization", pipeline.getRandomization());
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
            telemetry.addData("Distance to point", robotMovement.getDistanceToPoint());
            telemetry.addData("Angle to preferred", robotMovement.getAngleToPreferred());
            telemetry.addData("X", worldPosition.getxPosition());
            telemetry.addData("Y", worldPosition.getyPosition());
            telemetry.addData("Angle", worldPosition.getAngleDegrees());
            telemetry.update();

            switch (driveState){
                case WAIT_FOR_START:
                    robotMovement.setDrive(0);
                    robotMovement.setStrafe(0);
                    robotMovement.setRotate(0);
                    autonFunctions.prepShootHighGoal();
                    driveState = HighGoalOneWobbleStack.DRIVE_TO_SHOTS;
                    break;

                case DRIVE_TO_SHOTS:
                    robotMovement.goToPoint(-5, 60, 0);
                    if(worldPosition.getyPosition() > 35)
                        driveState = HighGoalOneWobbleStack.FIRST_SHOTS;
                    break;

                case FIRST_SHOTS:
                    robotMovement.goToPoint(-11, 57, -8, 1, 1.2);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.75))
                        aimTime.reset();
                    else if(aimed)
                        shotsFired += autonFunctions.shootOnceHigh();
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;

                    if(shotsFired < 3)
                        postShotTime.reset();
                    else if(postShotTime.milliseconds() > 250) {
                        if(randomization != Randomization.A)
                            driveState = HighGoalOneWobbleStack.BACK_TO_STACK;
                        else
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK:
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(-10, 40, -90, 1, 1.1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK;
                    }
                    break;

                case INTAKE_STACK:
                    aimed = false;
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    robotMovement.goToPointMaxPower(-22, 38, -90, .85, 1, .35);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.BACK_FROM_STACK;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case BACK_FROM_STACK:
                    robotMovement.goToPointMaxPower(-13, 40, -90, 1, 1, .75);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case SHOOT_HIGH:
                    robotMovement.goToPoint(-11, 59, -11.5, 1, 1);
                    if(!(robotMovement.getDistanceToPoint() < 2.5 && robotMovement.getAngleToPreferred() < 1.75))
                        aimTime.reset();
                    else if(aimed){
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 150)
                        aimed = true;

                    if(shotsFired == 1)
                        autonFunctions.turnOffIntake();

                    if(shotsFired < 3)
                        postShotTime.reset();
                    else if(postShotTime.milliseconds() > 250) {
                        if(randomization == Randomization.C)
                            driveState = HighGoalOneWobbleStack.BACK_TO_STACK_AGAIN;
                        else
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK_AGAIN:
                    shotsFired = 0;
                    aimed = false;
                    autonFunctions.turnOffShooter();
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(-7.5, 40, -90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK_AGAIN;
                    }
                    break;

                case INTAKE_STACK_AGAIN:
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    switch (stackJitter) {
                        case FORWARD_ONE:
                            robotMovement.goToPointMaxPower(-24, 40, -90, .8, 1, .35);
                            if(robotMovement.getDistanceToPoint() < 3)
                                stackJitter = StackJitter.BACK_ONE;
                            break;
                        case BACK_ONE:
                            robotMovement.goToPoint(-17, 40, -90);
                            if(robotMovement.getDistanceToPoint() < 2)
                                stackJitter = StackJitter.FORWARD_TWO;
                            break;
                        case FORWARD_TWO:
                            robotMovement.goToPointMaxPower(-31, 40, -90, 1, 1, .35);
                            if(robotMovement.getDistanceToPoint() < 2)
                                stackJitter = StackJitter.BACK_TWO;
                            break;
                        case BACK_TWO:
                            robotMovement.goToPoint(-25, 40, -90);
                            if(robotMovement.getDistanceToPoint() < 2)
                                stackJitter = StackJitter.FORWARD_THREE;
                            break;
                        case FORWARD_THREE:
                            robotMovement.goToPointMaxPower(-50, 35, -90, 1, 1, 5);
                            if (robotMovement.getDistanceToPoint() < 5) {
                                autonFunctions.prepShootHighGoalNoHopper();
                                driveState = HighGoalOneWobbleStack.BACK_FROM_STACK_AGAIN;
                            }
                    }
                    break;

                case BACK_FROM_STACK_AGAIN:
                    robotMovement.goToPoint(-17, 40, -90);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH_AGAIN;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case SHOOT_HIGH_AGAIN:
                    robotMovement.goToPoint(-11, 57, -4, 1, 1.2);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.75))
                        aimTime.reset();
                    else if(aimed)
                        shotsFired += autonFunctions.shootOnceHigh();
                    else if(aimTime.milliseconds() > 150)
                        aimed = true;

                    if(shotsFired < 5)
                        postShotTime.reset();
                    else if(postShotTime.milliseconds() > 250) {
                        driveState = randomization != Randomization.C ? HighGoalOneWobbleStack.PLACE_WOBBLE : HighGoalOneWobbleStack.APPROACH_WOBBLE;
                        autonFunctions.turnOffIntake();
                    }
                    break;

                case APPROACH_WOBBLE:
                    autonFunctions.actuallyTurnOffShooter();
                    robotMovement.goToPoint(-11, 200, 0);
                    if(worldPosition.getyPosition() > 70)
                        driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    break;

                case PLACE_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(-14, 58, 58, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(-18, 85, -25, 1, 1);
                    else
                        robotMovement.goToPoint(-8, 103, 25, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 5 && robotMovement.getAngleToPreferred() < 4) {
                        autonFunctions.depositWobble();
                        autonFunctions.turnOffShooter();
                    }
                    if(robotHardware.wobbleSecure.getPosition() == constants.wobbleSecureOpen)
                        driveState = HighGoalOneWobbleStack.PARK;
                    break;

                case PARK:
                    if(randomization == Randomization.A)
                        if(totalTime.seconds() > 27)
                            robotMovement.goToPoint(-30, 70, 0, .75, 1);
                        else
                            robotMovement.goToPoint(-30, 45, 0);
                    else
                        robotMovement.goToPoint(-10, 77, 0, 1, 1);
            }
        }
    }
}