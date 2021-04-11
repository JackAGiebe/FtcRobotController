package org.firstinspires.ftc.teamcode.autons;

import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomationizing.AutonFunctions;
import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.Randomization;
import org.firstinspires.ftc.teamcode.enums.drivestates.HighGoalTwoWobblesStack;
import org.firstinspires.ftc.teamcode.enums.drivestates.StackJitter;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCVExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Wall High Goal Two Wobbles Stack", group = "ACARRY")
public class BlueHighGoalTwoWobblesStack extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware(this);
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    AutonFunctions autonFunctions = new AutonFunctions(robotHardware, robotMovement);
    Constants constants = new Constants();

    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    HighGoalTwoWobblesStack driveState = HighGoalTwoWobblesStack.WAIT_FOR_START;
    StackJitter stackJitter = StackJitter.FORWARD_ONE;
    Randomization randomization = Randomization.A;

    int shotsFired = 0;

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime aimTime = new ElapsedTime();
    ElapsedTime postShotTime = new ElapsedTime();
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
                    driveState = HighGoalTwoWobblesStack.DRIVE_TO_SHOTS;
                    break;

                case DRIVE_TO_SHOTS:
                    robotMovement.goToPoint(5, 50,0, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 4)
                        driveState = HighGoalTwoWobblesStack.FIRST_SHOTS;
                    break;

                case FIRST_SHOTS:
                    robotMovement.goToPoint(5, 60, 20, .7, .8);
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
                            driveState = HighGoalTwoWobblesStack.BACK_TO_STACK;
                        else
                            driveState = HighGoalTwoWobblesStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK:
                    shotsFired = 0;
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(4, 42, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalTwoWobblesStack.INTAKE_STACK;
                    }
                    break;

                case INTAKE_STACK:
                    robotMovement.goToPointMaxPower(22, 39, 90, .75, 1, .3);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalTwoWobblesStack.BACK_FROM_STACK;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case BACK_FROM_STACK:
                    robotMovement.goToPoint(7, 39, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 4) {
                        driveState = HighGoalTwoWobblesStack.SHOOT_HIGH;
                    }
                    break;

                case SHOOT_HIGH:
                    robotMovement.goToPoint(5, 60, 20, .7, .8);
                    if(!(robotMovement.getDistanceToPoint() < 3 && robotMovement.getAngleToPreferred() < 2))
                        aimTime.reset();
                    else if(aimed){
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;
                    if(shotsFired < 2)
                        postShotTime.reset();
                    else if(postShotTime.milliseconds() > 250) {
                        if(randomization == Randomization.C)
                            driveState = HighGoalTwoWobblesStack.BACK_TO_STACK_AGAIN;
                        else
                            driveState = HighGoalTwoWobblesStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK_AGAIN:
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(4, 42, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 4) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalTwoWobblesStack.INTAKE_STACK_AGAIN;
                    }
                    break;

                case INTAKE_STACK_AGAIN:
                    if(stackJitter == StackJitter.FORWARD_ONE){
                        robotMovement.goToPointMaxPower(27, 39, 86, 1, 1, .4);
                        if(robotMovement.getDistanceToPoint() < 2)
                            stackJitter = StackJitter.BACK_ONE;
                    }
                    else if(stackJitter == StackJitter.BACK_ONE){
                        robotMovement.goToPointMaxPower(20, 39, 86, 1, 1, .7);
                        if(robotMovement.getDistanceToPoint() < 2)
                            stackJitter = StackJitter.FORWARD_TWO;
                    }
                    else if(stackJitter == StackJitter.FORWARD_TWO){
                        robotMovement.goToPointMaxPower(33, 39, 86, 1, 1, .4);
                        if(robotMovement.getDistanceToPoint() < 2)
                            stackJitter = StackJitter.BACK_TWO;
                    }
                    else if(stackJitter == StackJitter.BACK_TWO){
                        robotMovement.goToPointMaxPower(25, 39, 86, 1, 1, .7);
                        if(robotMovement.getDistanceToPoint() < 2)
                            stackJitter = StackJitter.FORWARD_THREE;
                    }
                    else if(stackJitter == StackJitter.FORWARD_THREE){
                        robotMovement.goToPointMaxPower(40, 39, 86, 1, 1, .6);
                        if(robotMovement.getDistanceToPoint() < 3) {
                            driveState = HighGoalTwoWobblesStack.BACK_FROM_STACK_AGAIN;
                        }
                    }
                    break;


                case BACK_FROM_STACK_AGAIN:
                    robotMovement.goToPoint(15, 42, 90);
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = HighGoalTwoWobblesStack.SHOOT_HIGH_AGAIN;
                    break;

                case SHOOT_HIGH_AGAIN:
                    robotMovement.goToPoint(5, 60, 20, .7, .8);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2))
                        aimTime.reset();
                    else if(aimed){
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;
                    if(shotsFired < 4)
                        postShotTime.reset();
                    else if(postShotTime.milliseconds() > 250) {
                        driveState = randomization != Randomization.C ? HighGoalTwoWobblesStack.PLACE_WOBBLE : HighGoalTwoWobblesStack.APPROACH_WOBBLE;
                        autonFunctions.turnOffIntake();
                    }
                    break;

                case APPROACH_WOBBLE:
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    robotMovement.goThroughPoint(-5, 90, 0, 1);
                    if(worldPosition.getyPosition() > 88)
                        driveState = HighGoalTwoWobblesStack.PLACE_WOBBLE;
                    break;

                case PLACE_WOBBLE:
                    autonFunctions.actuallyTurnOffShooter();
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(15, 77, -74, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(21, 84, 26, 1, 1);
                    else
                        robotMovement.goToPoint(5, 107, -12, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2) {
                        autonFunctions.depositWobble();
                        autonFunctions.turnOffShooter();
                    }
                    if(robotHardware.wobbleSecure.getPosition() != constants.wobbleSecureClosed)
                        driveState = HighGoalTwoWobblesStack.BACK_FROM_WOBBLE;
                    break;

                case BACK_FROM_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(18, 76, -77, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(17, 80, 20, 1, 1);
                    else
                        robotMovement.goToPoint(8, 104, -12, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2){
                        autonFunctions.liftWobbleArm();
                        driveState = randomization != Randomization.C ? HighGoalTwoWobblesStack.GO_TO_BACK_OF_FIELD : HighGoalTwoWobblesStack.PARK;
                    }
                    break;

                case GO_TO_BACK_OF_FIELD:
                    if(randomization == Randomization.B)
                        robotMovement.goToPoint(15, 0, 0, 5, 1);
                    else if(randomization == Randomization.A)
                        robotMovement.goToPoint(15,0, 0, 5, 1);
                    else
                        robotMovement.goToPoint(15, 0, 5, 5, 1);
                    if(worldPosition.getyPosition() < 38) {
                        driveState = HighGoalTwoWobblesStack.GO_TO_SECOND_WOBBLE;
                        autonFunctions.prepGrabWobble();
                    }
                    break;

                case GO_TO_SECOND_WOBBLE:
                    if(randomization == Randomization.B)
                        robotMovement.goToPoint(40, 0, 115, 1, .9);
                    else if(randomization == Randomization.A)
                        robotMovement.goToPoint(40, 0, 115, 1, .9);
                    else
                        robotMovement.goToPoint(40, 0, 115, 1, .9);
                    if(worldPosition.getyPosition() < 33) {
                        driveState = HighGoalTwoWobblesStack.GRAB_SECOND_WOBBLE;
                    }
                    break;

                case GRAB_SECOND_WOBBLE:
                    if(randomization != Randomization.C)
                        robotMovement.goToPointMaxPower(22, 13,140, 1, .5, .6);
                    else
                        robotMovement.goToPointMaxPower(24, 14.5, 140, 1, .5, .75);
                    if(robotMovement.getDistanceToPoint() < 3 && robotMovement.getAngleToPreferred() < 2)
                        autonFunctions.grabWobble();
                    if(robotHardware.wobble1.getPosition() == constants.wobble1Holding)
                        driveState = HighGoalTwoWobblesStack.BACK_FROM_WOBBLE_GRAB;
                    break;

                case BACK_FROM_WOBBLE_GRAB:
                    robotMovement.goToPoint(12, 25, 150, 2, 1);
                    if(worldPosition.getyPosition() > 18)
                        driveState = randomization != Randomization.C ? HighGoalTwoWobblesStack.PLACE_SECOND_WOBBLE : HighGoalTwoWobblesStack.APPROACH_SECOND_WOBBLE;
                    break;

                case APPROACH_SECOND_WOBBLE:
                    robotMovement.goThroughPoint(-5, 70, 0, 1);
                    if(worldPosition.getyPosition() > 60)
                        driveState = HighGoalTwoWobblesStack.PLACE_SECOND_WOBBLE;
                    break;

                case PLACE_SECOND_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(15, 77, -74, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(15, 88, 26, 1, 1);
                    else
                        robotMovement.goToPoint(-5, 100, -12, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2) {
                        autonFunctions.depositWobble();
                        autonFunctions.turnOffShooter();
                    }
                    if(robotHardware.wobbleSecure.getPosition() != constants.wobbleSecureClosed)
                        driveState = HighGoalTwoWobblesStack.BACK_FROM_SECOND_WOBBLE;
                    break;

                case BACK_FROM_SECOND_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(18, 76, -77, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(17, 80, 20, 1, 1);
                    else
                        robotMovement.goToPoint(-5, 104, -12, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2){
                        autonFunctions.liftWobbleArm();
                        driveState = HighGoalTwoWobblesStack.PARK;
                    }
                    break;

                case PARK:
                    if(randomization != randomization.A)
                        robotMovement.goToPoint(5, 76, 0, 1, 1);
                    else {
                        if(totalTime.seconds() > 0)
                            robotMovement.goToPoint(18, 76, 0, 1, 1);
                        else
                            robotMovement.goToPoint(18, 45, 0, 1, 1);
                    }
                    break;
            }
        }
    }
}