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
    Constants constants = new Constants();
    MathFunctions mathFunctions = new MathFunctions();
    ElapsedTime pushWait = new ElapsedTime();

    public AutonFunctions(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
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

    public int shotsFired = 0;
    //needs to reset shotsFired to zero before being called
//    public boolean shootHighGoal() {
//        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
//        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
//        robotHardware.hopper1.setPosition(constants.hopper1Up);
//        robotHardware.hopper2.setPosition(constants.hopper2Up);
//
//        if(shotsFired < 3) {
//            if(pushStep == PushStep.NOT_MOVING) {
//                pushStep = PushStep.STEP_ONE;
//                pusherTimer.reset();
//            }
//            else if (pushStep == PushStep.STEP_ONE) {
//                robotHardware.pusher.setPosition(constants.pusherIn);
//                if (pusherTimer.milliseconds() > 200) {
//                    pushStep = PushStep.STEP_TWO;
//                    pusherTimer.reset();
//                }
//            }
//            else if (pushStep == PushStep.STEP_TWO) {
//                robotHardware.pusher.setPosition(constants.pusherOut);
//                if(shotsFired < 4) {
//                    if (robotHardware.shooter1.getVelocity() > 1575)
//                        pushStep = PushStep.NOT_MOVING;
//                }
//                else{
//                    if(robotHardware.shooter1.getVelocity() > 1625)
//                        pushStep = PushStep.NOT_MOVING;
//                }
//            }
//            else
//                robotHardware.pusher.setPosition(constants.pusherOut);
//            return true;
//        }
//        else
//            return false;
//    }

    public int shootOnceHigh(){
        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);

        if(pushStep == PushStep.NOT_MOVING) {
            robotHardware.pusher.setPosition(constants.pusherOut);
            if(shotTimer.seconds() > 3) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }
            return 0;
        }
        else if (pushStep == PushStep.STEP_ONE) {
            robotHardware.pusher.setPosition(constants.pusherIn);
            if (pusherTimer.milliseconds() > 200) {
                shotTimer.reset();
                pushStep = PushStep.NOT_MOVING;
                return 1;
            }
            else
                return 0;
        }
        else
            return 0;
    }



    public int shootOnceLow(){
        robotHardware.shooter1.setVelocity(constants.shootHighVelocity);
        robotHardware.shooter2.setVelocity(constants.shootHighVelocity);
        robotHardware.hopper1.setPosition(constants.hopper1Up);
        robotHardware.hopper2.setPosition(constants.hopper2Up);

        if(pushStep == PushStep.NOT_MOVING) {
            robotHardware.pusher.setPosition(constants.pusherOut);
            if(shotTimer.seconds() > 3) {
                pushStep = PushStep.STEP_ONE;
                pusherTimer.reset();
            }
            return 0;
        }
        else if (pushStep == PushStep.STEP_ONE) {
            robotHardware.pusher.setPosition(constants.pusherIn);
            if (pusherTimer.milliseconds() > 200) {
                shotTimer.reset();
                pushStep = PushStep.NOT_MOVING;
                return 1;
            }
            else
                return 0;
        }
        else
            return 0;
    }

    public void turnOffShooter(){
        robotHardware.shooter1.setPower(0);
        robotHardware.shooter2.setPower(0);
    }

    public void depositWobble(){
        robotHardware.wobble1.setPosition(constants.wobble1Front);
        robotHardware.wobble2.setPosition(constants.wobble2Front);
        robotHardware.wobbleSecure.setPosition(constants.wobbleSecureOpen);
    }

    public ElapsedTime wobbleTime = new ElapsedTime();
    //needs wobbleTime reset before being called
    public boolean grabWobble(){
        robotHardware.wobble1.setPosition(constants.wobble1Front);
        robotHardware.wobble2.setPosition(constants.wobble2Front);
        robotHardware.wobbleSecure.setPosition(constants.wobbleSecureClosed);
        if(wobbleTime.seconds() > 200) {
            robotHardware.wobble1.setPosition(constants.wobble1Holding);
            robotHardware.wobble2.setPosition(constants.wobble2Holding);
        }
        if(wobbleTime.seconds() > 500)
            return false;
        else
            return true;
    }

    public void resetShotTimer(){
        shotTimer.reset();
    }
}