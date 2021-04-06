package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.PushStep;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

public class AutonFunctions {

    RobotHardware robotHardware;
    RobotMovement robotMovement;
    Constants constants = new Constants();
    MathFunctions mathFunctions = new MathFunctions();
    ElapsedTime pushWait = new ElapsedTime();
    ElapsedTime aimTime = new ElapsedTime();

    public AutonFunctions(RobotHardware robotHardware, RobotMovement robotMovement) {
        this.robotHardware = robotHardware;
        this.robotMovement = robotMovement;
    }

    PushStep pushStep = PushStep.NOT_MOVING;
    ElapsedTime pusherTimer = new ElapsedTime();
    public ElapsedTime shotTimer = new ElapsedTime();

    public void prepShootHighGoal(){
        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);
    }

    public void prepShootHighGoalNoHopper(){
        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
    }

    public void prepShootLowGoal(){
        robotHardware.shooter1.setVelocity(constants.shootLowVelocity);
        robotHardware.shooter2.setVelocity(constants.shootLowVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);
    }

    public int shootOnceHigh() {
        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);

        if (pushStep == PushStep.NOT_MOVING) {
            robotHardware.pusher.setPosition(constants.pusherOut);
            if (robotHardware.shooter1.getVelocity() >= constants.shootHighVelocity - 10) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }
            return 0;
        } else if (pushStep == PushStep.STEP_ONE) {
            robotHardware.pusher.setPosition(constants.pusherIn);
            if (pusherTimer.milliseconds() > 80) {
                pusherTimer.reset();
                pushStep = PushStep.STEP_TWO;
                return 1;
            }
            else
                return 0;
        }
        else if (pushStep == PushStep.STEP_TWO){
            robotHardware.pusher.setPosition(constants.pusherOut);
            if(pusherTimer.milliseconds() > 120){
                pushStep = PushStep.NOT_MOVING;
            }
            return 0;
        }
        else
            return 0;
    }



    public int shootOnceLow(){
        robotHardware.shooter1.setVelocity(constants.shootLowVelocity);
        robotHardware.shooter2.setVelocity(constants.shootLowVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);

        if (pushStep == PushStep.NOT_MOVING) {
            robotHardware.pusher.setPosition(constants.pusherOut);
            if (robotHardware.shooter1.getVelocity() >= constants.shootLowVelocity - 10) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }
            return 0;
        } else if (pushStep == PushStep.STEP_ONE) {
            robotHardware.pusher.setPosition(constants.pusherIn);
            if (pusherTimer.milliseconds() > 200) {
                pusherTimer.reset();
                pushStep = PushStep.STEP_TWO;
                return 1;
            }
            else
                return 0;
        }
        else if (pushStep == PushStep.STEP_TWO){
            robotHardware.pusher.setPosition(constants.pusherOut);
            if(pusherTimer.milliseconds() > 300){
                pushStep = PushStep.NOT_MOVING;
            }
            return 0;
        }
        else
            return 0;
    }

    public void autonStart(){
        robotHardware.wobbleSecure.setPosition(constants.wobbleSecureClosed);
        robotHardware.wobble1.setPosition(constants.wobble1Holding);
        robotHardware.wobble2.setPosition(constants.wobble2Holding);
        robotHardware.pusher.setPosition(constants.pusherOut);
    }

    public void turnOnIntake(){
        robotHardware.intake1.setPower(constants.intakePower);
        robotHardware.intake2.setPower(constants.intakePower);
        robotHardware.hopper1.setPosition(constants.hopper1Down);
        robotHardware.hopper2.setPosition(constants.hopper2Down);
    }

    public void turnOffIntake(){
        robotHardware.intake1.setPower(0);
        robotHardware.intake2.setPower(0);
    }

    public void turnOffShooter(){
        robotHardware.shooter1.setPower(0);
        robotHardware.shooter2.setPower(0);
    }

    public void depositWobble(){
        if(robotHardware.wobble1.getPosition() != constants.wobble1Front)
            wobbleTime.reset();
        robotHardware.wobble1.setPosition(constants.wobble1Front);
        robotHardware.wobble2.setPosition(constants.wobble2Front);
        if(wobbleTime.milliseconds() > 400)
            robotHardware.wobbleSecure.setPosition(constants.wobbleSecureOpen);
    }
    public void liftWobbleArm(){
        robotHardware.wobble1.setPosition(constants.wobble1Back);
        robotHardware.wobble2.setPosition(constants.wobble2Back);
        robotHardware.wobbleSecure.setPosition(constants.wobbleSecureOpen);
    }

    public ElapsedTime wobbleTime = new ElapsedTime();
    //needs wobbleTime reset before being called
    public void grabWobble(){
        if(robotHardware.wobbleSecure.getPosition() != constants.wobbleSecureClosed)
            wobbleTime.reset();
        robotHardware.wobbleSecure.setPosition(constants.wobbleSecureClosed);
        if(wobbleTime.milliseconds() > 400){
            robotHardware.wobble1.setPosition(constants.wobble1Holding);
            robotHardware.wobble2.setPosition(constants.wobble2Holding);
        }
    }
}