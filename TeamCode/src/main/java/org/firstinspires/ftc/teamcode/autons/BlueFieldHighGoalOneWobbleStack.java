package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Blue Field High Goal One Wobble Stack", group = "BNB")
public class BlueFieldHighGoalOneWobbleStack extends LinearOpMode {
    RobotHardware robotHardware = new RobotHardware(this);
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    AutonFunctions autonFunctions = new AutonFunctions(robotHardware, robotMovement);
    Constants constants = new Constants();

    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    HighGoalOneWobbleStack driveState = HighGoalOneWobbleStack.WAIT_FOR_START;
    StackJitter stackJitter = StackJitter.FORWARD_ONE;
    Randomization randomization = Randomization.A;

    int shotsFired = 0;

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime aimTime = new ElapsedTime();
    boolean aimed = false;

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


            telemetry.addData("Aimed", aimed);
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
                    robotMovement.goToPoint(50, 55, 0, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 4)
                        driveState = HighGoalOneWobbleStack.FIRST_SHOTS;
                    break;

                case FIRST_SHOTS:
                    robotMovement.goToPoint(45, 59, -10, .9, 1.3);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5)
                        shotsFired += autonFunctions.shootOnceHigh();
                    if(shotsFired == 3) {
                        if(randomization != Randomization.A)
                            driveState = HighGoalOneWobbleStack.BACK_TO_STACK;
                        else
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK:
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(45, 40, -90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK;
                    }
                    break;

                case INTAKE_STACK:
                    shotsFired = 0;
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    robotMovement.goToPointMaxPower(30, 44, -90, 1, 1, .35);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.BACK_FROM_STACK;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case BACK_FROM_STACK:
                    robotMovement.goToPoint(43, 40, -90, 1, 1);
                    shotsFired = 0;
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH;
                    break;

                case SHOOT_HIGH:
                    robotMovement.goToPoint(45, 59, -10, 1, 1.3);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5) {
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    if(shotsFired == 1) {
                        if(randomization == Randomization.C)
                            driveState = HighGoalOneWobbleStack.BACK_TO_STACK_AGAIN;
                        else
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    }
                    break;

                case BACK_TO_STACK_AGAIN:
                    shotsFired = 0;
                    autonFunctions.turnOffShooter();
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(45, 42, -90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK_AGAIN;
                    }
                    break;

                case INTAKE_STACK_AGAIN:
                    if(stackJitter == StackJitter.FORWARD_ONE){
                        robotMovement.goToPointMaxPower(22, 39, -90, 1, 1, .4);
                        if(robotMovement.getDistanceToPoint() < 3) {
                            stackJitter = StackJitter.BACK_ONE;
                        }
                    }
                    if(stackJitter == StackJitter.BACK_ONE){
                        robotMovement.goToPointMaxPower(27, 39, -90, 1, 1, .4);
                        if(robotMovement.getDistanceToPoint() < 3) {
                            stackJitter = StackJitter.FORWARD_TWO;
                        }
                    }
                    if(stackJitter == StackJitter.FORWARD_TWO) {
                        robotMovement.goToPointMaxPower(3, 39, -90, 1, 1, .35);
                        if (robotMovement.getDistanceToPoint() < 3) {
                            driveState = HighGoalOneWobbleStack.BACK_FROM_STACK_AGAIN;
                            autonFunctions.prepShootHighGoalNoHopper();
                        }
                    }
                    break;

                case BACK_FROM_STACK_AGAIN:
                    robotMovement.goToPoint(35, 35, -90);
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH_AGAIN;
                    break;

                case SHOOT_HIGH_AGAIN:
                    robotMovement.goToPoint(48, 59, -10, 1, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5))
                        aimTime.reset();
                    else if(aimed) {
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;
                    if(shotsFired == 4 || totalTime.seconds() > 24)
                        driveState = HighGoalOneWobbleStack.APPROACH_WOBBLE;
                    break;

                case APPROACH_WOBBLE:
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(48, 110, 0, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 4)
                        driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    break;

                case PLACE_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(14, 76, -72, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(37, 87, -33, 1, 1);
                    else
                        robotMovement.goToPoint(12, 120, -90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2) {
                        autonFunctions.depositWobble();
                        autonFunctions.turnOffShooter();
                    }
                    if(robotHardware.wobbleSecure.getPosition() == constants.wobbleSecureOpen)
                        driveState = HighGoalOneWobbleStack.BACK_FROM_WOBBLE;
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
                        driveState = HighGoalOneWobbleStack.PARK;
                    }
                    break;

                case PARK:
                    robotMovement.goToPoint(48, 76, 0, 1, 1);
            }
        }
    }
}