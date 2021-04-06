package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomationizing.AutonFunctions;
import org.firstinspires.ftc.teamcode.autonomationizing.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomationizing.WorldPosition;
import org.firstinspires.ftc.teamcode.enums.Randomization;
import org.firstinspires.ftc.teamcode.enums.drivestates.HighGoalOneWobbleStack;
import org.firstinspires.ftc.teamcode.enums.drivestates.PowershotOneWobbleStack;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCVExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "Blue Power Shot One Wobble", group = "BNB")
public class BluePowerShotOneWobble extends LinearOpMode {

    RobotHardware robotHardware = new RobotHardware(this);
    WorldPosition worldPosition = new WorldPosition(0, 0, 0, robotHardware, this);
    RobotMovement robotMovement = new RobotMovement(this, robotHardware, worldPosition);
    AutonFunctions autonFunctions = new AutonFunctions(robotHardware, robotMovement);
    Constants constants = new Constants();

    PowershotOneWobbleStack driveState = PowershotOneWobbleStack.WAIT_FOR_START;
    Randomization randomization = Randomization.A;

    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    int powerShotsHit = 0;
    int shotsFired = 0;
    boolean aimed = false;

    ElapsedTime totalTime = new ElapsedTime();
    ElapsedTime aimTime = new ElapsedTime();

    @Override
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
            telemetry.addData("Aimed", aimed);
            telemetry.addData("Power Shots Hit", powerShotsHit);
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
                    driveState = PowershotOneWobbleStack.DRIVE_TO_SHOTS;
                    break;

                case DRIVE_TO_SHOTS:
                    driveState = PowershotOneWobbleStack.FIRST_POWER_SHOT;
                    break;

                case FIRST_POWER_SHOT:
                    robotMovement.goToPoint(45, 59, 11, 1.2, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5))
                        aimTime.reset();
                    else if(aimed)
                        powerShotsHit += autonFunctions.shootOnceLow();
                    else if(aimTime.milliseconds() > 750)
                        aimed = true;
                    if(powerShotsHit == 1) {
                        robotHardware.pusher.setPosition(constants.pusherOut);
                        driveState = PowershotOneWobbleStack.SECOND_POWER_SHOT;
                    }
                    break;

                case SECOND_POWER_SHOT:
                    robotMovement.goToPoint(45, 59, 7, 1.2, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5))
                        aimTime.reset();
                    else if(aimed)
                        powerShotsHit += autonFunctions.shootOnceLow();
                    else if(aimTime.milliseconds() > 750)
                        aimed = true;
                    if(powerShotsHit == 2) {
                        robotHardware.pusher.setPosition(constants.pusherOut);
                        driveState = PowershotOneWobbleStack.THIRD_POWER_SHOT;
                    }
                    break;

                case THIRD_POWER_SHOT:
                    robotMovement.goToPoint(45, 59, 3, 1.2, 1.3);
                    if(!(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1.5))
                        aimTime.reset();
                    else if(aimed)
                        powerShotsHit += autonFunctions.shootOnceLow();
                    else if(aimTime.milliseconds() > 750)
                        aimed = true;
                    if(powerShotsHit == 3) {
                        if(randomization != Randomization.C) {
                            driveState = PowershotOneWobbleStack.PLACE_WOBBLE;
                        }
                        else{
                            driveState = PowershotOneWobbleStack.APPROACH_WOBBLE;
                        }
                    }
                    break;

                case PLACE_WOBBLE:
                    if(randomization == Randomization.A)
                        robotMovement.goToPoint(10, 76, -72, 1, 1);
                    else if(randomization == Randomization.B)
                        robotMovement.goToPoint(34, 87, -33, 1, 1);
                    else
                        robotMovement.goToPoint(18, 119, -47, 1, 1);
                    if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 2)
                        autonFunctions.depositWobble();
                    if(robotHardware.wobbleSecure.getPosition() != constants.wobbleSecureClosed)
                        driveState = PowershotOneWobbleStack.BACK_FROM_WOBBLE;
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
