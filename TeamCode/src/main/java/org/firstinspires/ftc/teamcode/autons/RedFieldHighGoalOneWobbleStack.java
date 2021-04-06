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

@Autonomous(name = "Red Field High Goal One Wobble", group = "BNB")
public class RedFieldHighGoalOneWobbleStack extends LinearOpMode {
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
                    robotMovement.goToPoint(-43, 59, 0, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2)
                        driveState = HighGoalOneWobbleStack.FIRST_SHOTS;
                    break;


                case FIRST_SHOTS:
                    robotMovement.goToPoint(-43, 59, 19, 1, 1.5);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5))
                        aimTime.reset();
                    else if(aimed)
                        shotsFired += autonFunctions.shootOnceHigh();
                    else
                        if(aimTime.milliseconds() > 200)
                            aimed = true;
                    if(shotsFired == 3)
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    break;

                case BACK_TO_STACK:
                    shotsFired = 0;
                    autonFunctions.turnOnIntake();
                    robotMovement.goToPoint(-40, 38, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK;
                    }
                    break;

                case INTAKE_STACK:
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    robotMovement.goToPointMaxPower(-32, 41, 90, 1, 1, .5);
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = HighGoalOneWobbleStack.BACK_FROM_STACK;
                    break;

                case BACK_FROM_STACK:
                    robotMovement.goToPoint(-40, 35, 90, .6, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case SHOOT_HIGH:
                    robotMovement.goToPoint(-44, 61, 18.5, 1.2, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2))
                        aimTime.reset();
                    else if(aimed) {
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;
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
                    robotMovement.goToPoint(-30, 36, 90, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        autonFunctions.turnOffShooter();
                        driveState = HighGoalOneWobbleStack.INTAKE_STACK_AGAIN;
                    }
                    break;

                case INTAKE_STACK_AGAIN:
                    if(stackJitter == StackJitter.FORWARD_ONE){
                        robotMovement.goToPoint(-25, 40, 90);
                        if(robotMovement.getDistanceToPoint() < 3)
                            stackJitter = StackJitter.BACK_ONE;
                    }
                    else if(stackJitter == StackJitter.BACK_ONE){
                        robotMovement.goToPoint(-32, 40, 90);
                        if(robotMovement.getDistanceToPoint() < 3)
                            stackJitter = StackJitter.FORWARD_TWO;
                    }
                    else if(stackJitter == StackJitter.FORWARD_TWO){
                        robotMovement.goToPoint(-20, 40, 90);
                        if(robotMovement.getDistanceToPoint() < 3)
                            stackJitter = StackJitter.BACK_TWO;
                    }
                    else if(stackJitter == StackJitter.BACK_TWO){
                        robotMovement.goToPoint(-27, 40, 90);
                        if(robotMovement.getDistanceToPoint() < 3)
                            stackJitter = StackJitter.FORWARD_THREE;
                    }
                    else{
                        robotMovement.goToPoint(-10, 43, 90);
                        if(robotMovement.getDistanceToPoint() < 3) {
                            driveState = HighGoalOneWobbleStack.BACK_FROM_STACK_AGAIN;
                        }
                    }
                    break;

                case BACK_FROM_STACK_AGAIN:
                    robotMovement.goToPoint(-40, 35, 90, .6, 1);
                    if(robotMovement.getDistanceToPoint() < 3) {
                        driveState = HighGoalOneWobbleStack.SHOOT_HIGH_AGAIN;
                        autonFunctions.prepShootHighGoalNoHopper();
                    }
                    break;

                case SHOOT_HIGH_AGAIN:
                    robotMovement.goToPoint(-44, 61, 18.5, 1.2, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2))
                        aimTime.reset();
                    else if(aimed){
                        autonFunctions.turnOffIntake();
                        shotsFired += autonFunctions.shootOnceHigh();
                    }
                    else if(aimTime.milliseconds() > 200)
                        aimed = true;
                    if(shotsFired == 3) {
                        autonFunctions.turnOnIntake();
                        if(randomization != Randomization.C)
                            driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                        else
                            driveState = HighGoalOneWobbleStack.APPROACH_WOBBLE;
                    }
                    break;

                case APPROACH_WOBBLE:
                    robotMovement.goToPoint(-35, 120, 0, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 3)
                        driveState = HighGoalOneWobbleStack.PLACE_WOBBLE;
                    break;

                case PLACE_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(-14, 64, 76, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(-35, 83, 45, 1, 1);
                    else
                        robotMovement.goToPoint(-17, 120,90, 1.25, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2) {
                        autonFunctions.depositWobble();
                        autonFunctions.turnOffShooter();
                    }
                    if(robotHardware.wobbleSecure.getPosition() == constants.wobbleSecureOpen)
                        driveState = HighGoalOneWobbleStack.BACK_FROM_WOBBLE;
                    break;

                case BACK_FROM_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(-17, 63, 70, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(-28, 82, 38, 1, 1);
                    else
                        robotMovement.goToPoint(-25, 108, 70, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2){
                        autonFunctions.liftWobbleArm();
                        driveState = HighGoalOneWobbleStack.PARK;
                    }
                    break;

                case PARK:
                    autonFunctions.turnOffIntake();
                    robotMovement.goToPoint(-55, 77, 0, 1, 1);
                    break;
            }
        }
    }
}