package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.IntakeStatus;
import org.firstinspires.ftc.teamcode.enums.PowerShotsHit;
import org.firstinspires.ftc.teamcode.enums.PushStep;
import org.firstinspires.ftc.teamcode.enums.WobblePosition;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

import java.util.ArrayList;

public class TeleOpControls {

    //Shooter Adjust Variables
    public boolean tickUpPressed = false, tickUpHeld = false, tickDownPressed = false, tickDownHeld = false;

    //Drive Controls Variables
    public double drive, strafe, rotate;
    public boolean slowMode = false, slowModePressed = false, slowModeHeld = false;

    //Power Shot Variables
    public boolean overrideDrive = false, resetPositionHeld = false, resetPositionPressed = false;
    public ArrayList<PowerShotsHit> hits = new ArrayList<>();
    public boolean turnPressed = false, turnHeld = false, negTurnPressed = false, negTurnHeld = false;
    public double xYeet = -46, yYeet = 59, angleYeet = 7.5;
    ElapsedTime resetTimer = new ElapsedTime();

    //Shooter Controls Variables
    public boolean shooting = false, angleUp = true, hopperUp = false, shootPressed = false, angleUpPressed = false, angleDownPressed = false, hopperPressed = false,
            pushPressed = false, shootHeld = false, angleUpHeld = false, angleDownHeld = false, hopperHeld = false, pushHeld = false;
    public ElapsedTime pusherTimer = new ElapsedTime();
    public PushStep pushStep = PushStep.NOT_MOVING;

    //Intake Controls Variables
    public boolean intakePressed = false, intakeHeld = false;
    public IntakeStatus intakeStatus = IntakeStatus.INTAKING;

    //Wobble Controls Variables
    public boolean wobblePressed = false, wobbleHeld = false, wobbleSecurePressed = false, wobbleSecureHeld = false, wobbleSecured = false;
    public WobblePosition wobblePosition = WobblePosition.BACK;


    LinearOpMode op;
    RobotHardware robotHardware;
    WorldPosition worldPosition;
    RobotMovement robotMovement;
    Constants constants = new Constants();

    public TeleOpControls(RobotHardware robotHardware, LinearOpMode op, WorldPosition worldPosition, RobotMovement robotMovement){
        this.robotHardware = robotHardware;
        this.op = op;
        this.worldPosition = worldPosition;
        this.robotMovement = robotMovement;
    }

    public void driveControls(){
        drive = -op.gamepad1.left_stick_y;
        strafe = -op.gamepad1.left_stick_x;
        rotate = op.gamepad1.right_trigger - op.gamepad1.left_trigger;

        if(!slowModeHeld){
            if(op.gamepad1.left_stick_button){
                slowModePressed = true;
                slowModeHeld = true;
            }
            else
                slowModePressed = false;
        }
        else{
            slowModePressed = false;
            if(!op.gamepad1.left_stick_button)
                slowModeHeld = false;
        }


        if(shooting || wobblePosition == WobblePosition.FORWARD){
            slowMode = true;
        }
        if(hopperPressed || wobblePressed){
            slowMode = false;
        }

        if(slowModePressed)
            slowMode = !slowMode;

        if(!overrideDrive) {
            if (!slowMode) {
                robotHardware.frontLeft.setPower(drive + strafe + rotate);
                robotHardware.frontRight.setPower(drive + strafe - rotate);
                robotHardware.backLeft.setPower(drive - strafe + rotate);
                robotHardware.backRight.setPower(drive - strafe - rotate);
            } else {
                robotHardware.frontLeft.setPower((drive + strafe + rotate) / 2);
                robotHardware.frontRight.setPower((drive + strafe - rotate) / 2);
                robotHardware.backLeft.setPower((drive - strafe + rotate) / 2);
                robotHardware.backRight.setPower((drive - strafe - rotate) / 2);
            }
        }
        else
            robotMovement.setMotorPowers();
    }

    public void driveForPowerShots(Alliance alliance){
        if(!resetPositionHeld){
            if(op.gamepad1.dpad_down && resetTimer.seconds() > 10){
                resetPositionPressed = true;
                resetPositionHeld = true;
                resetTimer.reset();
            }
            else
                resetPositionPressed = false;
        }
        else{
            resetPositionPressed = false;
            if(!op.gamepad1.dpad_down)
                resetPositionHeld = false;
        }

        if(resetPositionPressed) {
            worldPosition.setxPosition(0);
            worldPosition.setyPosition(0);
            worldPosition.setAngleDegrees(180);
        }

        if(alliance == Alliance.RED){
            if(op.gamepad1.dpad_up){
                overrideDrive = true;
                robotMovement.goToPoint(-48, 59, 0, 1.75, 1.5);
            }
            else if(op.gamepad1.dpad_right){
                overrideDrive = true;
                robotMovement.goToPoint(-44, 59, 0, 1.75, 1.5);
            }
            else if(op.gamepad1.dpad_left){
                overrideDrive = true;
                robotMovement.goToPoint(-51, 59, -2, 1.75, 1.5);
            }
            else
                overrideDrive = false;
        }
        else{
            if(op.gamepad1.dpad_up){
                overrideDrive = true;
                robotMovement.goToPoint(52, 59, 6, 1.75, 1.5);
            }
            else if(op.gamepad1.dpad_right){
                overrideDrive = true;
                robotMovement.goToPoint(57, 60, 7, 1.75, 1.5);
            }
            else if(op.gamepad1.dpad_left){
                overrideDrive = true;
                robotMovement.goToPoint(48, 59, 6, 1.75, 1.5);
            }
            else
                overrideDrive = false;
        }
    }

    public void turnMoveForPowerShots(){
        if(!turnHeld){
            if(op.gamepad1.right_stick_x < -.5){
                turnHeld = true;
                turnPressed = true;
            }
            else
                turnPressed = true;
        }
        else{
            turnPressed = false;
            if(!(op.gamepad1.right_stick_x < -.5))
                turnHeld = false;
        }

        if(!negTurnHeld){
            if(op.gamepad1.right_stick_x > .5){
                negTurnHeld = true;
                negTurnPressed = true;
            }
            else
                negTurnPressed = true;
        }
        else{
            negTurnPressed = false;
            if(!(op.gamepad1.right_stick_x > .5))
                negTurnHeld = false;
        }

        if(turnPressed) {
            xYeet = worldPosition.getxPosition() + 2;
            yYeet = worldPosition.getyPosition();
            angleYeet = worldPosition.getAngleDegrees() + 4;
        }
        else if(turnPressed){
            xYeet = worldPosition.getxPosition() - 2;
            yYeet = worldPosition.getyPosition();
            angleYeet = worldPosition.getAngleDegrees() - 4;
        }
        if(turnHeld || negTurnHeld){
            overrideDrive = true;
            robotMovement.goToPoint(xYeet, yYeet, angleYeet, 1, 1);
        }
    }

    public boolean fullAutoPowerShots(Alliance alliance){
        if(Math.abs(op.gamepad1.right_stick_x) + Math.abs(op.gamepad1.right_stick_y) > .5 && (!hits.contains(PowerShotsHit.LEFT) || !hits.contains(PowerShotsHit.MIDDLE) || !hits.contains(PowerShotsHit.RIGHT))){
            robotHardware.shooter1.setVelocity(constants.shootLowVelocity);
            robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
            robotHardware.hopper1.setPosition(constants.hopper1Up);
            robotHardware.hopper2.setPosition(constants.hopper2Up);

            if(alliance == Alliance.RED) {
                if (!hits.contains(PowerShotsHit.LEFT)) {
                    robotMovement.goToPoint(-48, 59, -3, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.LEFT);
                } else if (!hits.contains(PowerShotsHit.MIDDLE)) {
                    robotMovement.goToPoint(-46, 59, 3, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.MIDDLE);
                } else if (!hits.contains(PowerShotsHit.RIGHT)) {
                    robotMovement.goToPoint(-46, 59, 11, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.RIGHT);
                }
            }
            else{
                if (!hits.contains(PowerShotsHit.RIGHT)) {
                    robotMovement.goToPoint(45, 59, 11, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.RIGHT);
                } else if (!hits.contains(PowerShotsHit.MIDDLE)) {
                    robotMovement.goToPoint(45, 59, 7, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.MIDDLE);
                } else if (!hits.contains(PowerShotsHit.LEFT)) {
                    robotMovement.goToPoint(45, 59, 3, 1, 1);
                    if (pushStep == PushStep.STEP_TWO)
                        hits.add(PowerShotsHit.LEFT);
                }
            }

            if(robotMovement.getDistanceToPoint() < 2 && robotMovement.getAngleToPreferred() < 1){
                if (pushStep == PushStep.NOT_MOVING) {
                    pushStep = PushStep.STEP_ONE;
                    pusherTimer.reset();
                }
                if (pushStep == PushStep.STEP_ONE) {
                    robotHardware.pusher.setPosition(constants.pusherIn);
                    if (pusherTimer.milliseconds() > 150) {
                        pusherTimer.reset();
                        pushStep = PushStep.STEP_TWO;
                    }
                } else if (pushStep == PushStep.STEP_TWO) {
                    robotHardware.pusher.setPosition(constants.pusherOut);
                    if (pusherTimer.milliseconds() > 250)
                        pushStep = PushStep.NOT_MOVING;
                } else
                    robotHardware.pusher.setPosition(constants.pusherOut);
            }
            return true;
        }
        else
            return false;
    }

    public void shooterControls(){
        //fly wheel (automatically turns on when hopper goes up and turns off when hopper goes down)
        if(hopperUp)
            shooting = true;
        else
            shooting = false;

        if(overrideDrive)
            angleUp = false;

        if(shooting){
            if(angleUp) {
                robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
                robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
            }
            else{
                robotHardware.shooter1.setVelocity(constants.shootLowVelocity);
                robotHardware.shooter2.setVelocity(constants.shootLowVelocity);
            }
        }
        else{
            robotHardware.shooter1.setPower(0);
            robotHardware.shooter2.setPower(0);
        }

        //angle
        if(!angleDownHeld){
            if(op.gamepad1.left_bumper){
                angleDownPressed = true;
                angleDownHeld = true;
            }
            else
                angleDownPressed=false;
        }
        else{
            angleDownPressed=false;
            if(!op.gamepad1.left_bumper)
                angleDownHeld = false;
        }

        if(!angleUpHeld){
            if(op.gamepad1.right_bumper){
                angleUpPressed = true;
                angleUpHeld = true;
            }
            else
                angleUpPressed=false;
        }
        else{
            angleUpPressed=false;
            if(!op.gamepad1.right_bumper)
                angleUpHeld = false;
        }

        if(angleUpPressed)
            angleUp = true;
        else if(angleDownPressed)
            angleUp = false;


        //hopper controls
        if(!hopperHeld){
            if(op.gamepad1.a){
                hopperHeld = true;
                hopperPressed = true;
            }
            else
                hopperPressed = false;
        }
        else{
            hopperPressed = false;
            if(!op.gamepad1.a)
                hopperHeld = false;
        }

        if(hopperPressed){
            if (hopperUp)
                intakeStatus = IntakeStatus.INTAKING;
            else{
                intakeStatus = IntakeStatus.STOP;
            }
            hopperUp = !hopperUp;
        }

        if(hopperUp){
            robotHardware.hopper1.setPosition(constants.hopper1Up);
            robotHardware.hopper2.setPosition(constants.hopper2Up);
        }
        else{
            robotHardware.hopper1.setPosition(constants.hopper1Down);
            robotHardware.hopper2.setPosition(constants.hopper2Down);
        }

        //pusher
        if(angleUp && !overrideDrive) {
            if (!pushHeld) {
                if (op.gamepad1.x) {
                    if (robotHardware.shooter1.getVelocity() >= constants.shootHighVelocity && pushStep == PushStep.NOT_MOVING) {
                        pushPressed = true;
                        if(!angleUp)
                            pushHeld = true;
                    }
                } else
                    pushPressed = false;
            } else {
                pushPressed = false;
                if (!op.gamepad1.x)
                    pushHeld = false;
            }

            if(robotHardware.shooter1.getVelocity() < constants.shootHighVelocity)
                pushPressed = false;

            if (pushPressed && pushStep == PushStep.NOT_MOVING) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }

            if (pushStep == PushStep.STEP_ONE) {
                robotHardware.pusher.setPosition(constants.pusherIn);
                if (pusherTimer.milliseconds() > 150) {
                    pushStep = PushStep.STEP_TWO;
                    pusherTimer.reset();
                }
            }
            else if (pushStep == PushStep.STEP_TWO) {
                robotHardware.pusher.setPosition(constants.pusherOut);
                if (pusherTimer.milliseconds() > 250)
                    pushStep = PushStep.NOT_MOVING;
            }
            else
                robotHardware.pusher.setPosition(constants.pusherOut);
        }
        else {
            if (!pushHeld) {
                if (op.gamepad1.x) {
                    if (robotHardware.shooter1.getVelocity() >= constants.shootLowVelocity && pushStep == PushStep.NOT_MOVING) {
                        pushPressed = true;
                        pushHeld = false;
                    }
                } else
                    pushPressed = false;
            } else {
                pushPressed = false;
                if (!op.gamepad1.x)
                    pushHeld = false;
            }

            if(robotHardware.shooter1.getVelocity() < constants.shootLowVelocity){
                pushPressed = false;
            }

            if (pushPressed && pushStep == PushStep.NOT_MOVING) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }

            if (pushStep == PushStep.STEP_ONE) {
                robotHardware.pusher.setPosition(constants.pusherIn);
                if (pusherTimer.milliseconds() > 150) {
                    pusherTimer.reset();
                    pushStep = PushStep.STEP_TWO;
                }
            } else if (pushStep == PushStep.STEP_TWO) {
                robotHardware.pusher.setPosition(constants.pusherOut);
                if (pusherTimer.milliseconds() > 250)
                    pushStep = PushStep.NOT_MOVING;
            } else
                robotHardware.pusher.setPosition(constants.pusherOut);
        }}

    public void intakeControls(){
        if(!intakeHeld){
            if(op.gamepad1.y){
                intakePressed = true;
                intakeHeld = true;
            }
            else
                intakePressed = false;
        }
        else{
            intakePressed = false;
            if(!op.gamepad1.y)
                intakeHeld = false;
        }

        if(intakePressed){
            if(intakeStatus == IntakeStatus.STOP)
                intakeStatus = IntakeStatus.INTAKING;
            else if(intakeStatus == IntakeStatus.INTAKING)
                intakeStatus = IntakeStatus.REVERSE;
            else
                intakeStatus = IntakeStatus.STOP;
        }

        if(intakeStatus == IntakeStatus.INTAKING){
            robotHardware.intake1.setPower(constants.intakePower);
            robotHardware.intake2.setPower(constants.intakePower);
        }
        else if(intakeStatus == IntakeStatus.REVERSE){
            robotHardware.intake1.setPower(-constants.intakePower);
            robotHardware.intake2.setPower(-constants.intakePower);
        }
        else{
            robotHardware.intake1.setPower(0);
            robotHardware.intake2.setPower(0);
        }
    }

    public void wobbleControls(){
        //Wobble Movement
        if(!wobbleHeld){
            if(op.gamepad1.b){
                wobblePressed = true;
                wobbleHeld = true;
            }
            else
                wobblePressed = false;
        }
        else{
            wobblePressed = false;
            if (!op.gamepad1.b)
                wobbleHeld = false;
        }

        if(wobblePressed){
            if(wobblePosition != WobblePosition.FORWARD)
                wobblePosition = WobblePosition.FORWARD;
            else{
                if(wobbleSecured)
                    wobblePosition = WobblePosition.HOLDING;
                else
                    wobblePosition = WobblePosition.BACK;
            }
        }

        if(wobblePosition == WobblePosition.FORWARD){
            robotHardware.wobble1.setPosition(constants.wobble1Front);
            robotHardware.wobble2.setPosition(constants.wobble2Front);
        }
        else if(wobblePosition == WobblePosition.BACK){
            robotHardware.wobble1.setPosition(constants.wobble1Back);
            robotHardware.wobble2.setPosition(constants.wobble2Back);
        }
        else{
            robotHardware.wobble1.setPosition(constants.wobble1Holding);
            robotHardware.wobble2.setPosition(constants.wobble2Holding);
        }

        //Wobble Secure
        if(!wobbleSecureHeld){
            if(op.gamepad1.right_stick_button){
                wobbleSecurePressed = true;
                wobbleSecureHeld = true;
            }
            else
                wobbleSecurePressed = false;
        }
        else{
            wobbleSecurePressed = false;
            if(!op.gamepad1.right_stick_button)
                wobbleSecureHeld = false;
        }

        if(wobbleSecurePressed)
            wobbleSecured = !wobbleSecured;

        if(wobbleSecured)
            robotHardware.wobbleSecure.setPosition(constants.wobbleSecureClosed);
        else
            robotHardware.wobbleSecure.setPosition(constants.wobbleSecureOpen);

    }
}
