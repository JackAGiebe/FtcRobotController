package org.firstinspires.ftc.teamcode.stuffs;


public class Constants {


    public final String vuforiaKey = "ARnInK7/////AAABmT/cGj9enUxklocU1u44tiMvUkh9g7RrgGrwx843MuKXzVVSu586ftxs9Y0mFOG1HVO5Fn6X0P+uMfUPBU5yEPwTuIWklDDHtmeXEOmt10qJIa7H5XkJf4GqOMyOYNzC+so019AdOZ/ngP163cyaxwhKtl9qh/J4e5Kwzs2TPOWVC7M4u1qYNkGUBrF1kcYQjCHHQslggiJpFp0CUEePE5Mau6IDBYcMuMTEx1WW9dVpF0RF+hQnIeYrc2p3BC3cIohwIfHNwHSF0+DC1I8JbQRH3F8G0v/92wh8fRMvu2VE9Z9bf+Yunn0J3AL3wLpq7GX48U8H6z3TiKqr0x/0S84dekd6EUSUTE7cnEeZm8wZ";
    //in case we ever need to use the number 3
    public final int pi = (int) Math.PI;

    public double countsPerRevolution = 8192;
    public double wheelDiameter = 2, wheelCircumference = wheelDiameter * Math.PI;
    public double countsPerInch = 8192 / wheelCircumference;

    public double wheelBaseSeperation = 15.05, normalDifferenceRadians = 10800/Math.PI;

    public double wobbleSecureOpen = .5, wobbleSecureClosed = 0;
    public double wobble1Holding = .7, wobble1Front = .05, wobble1Back = .85, wobble2Holding = 0.3, wobble2Front = .95, wobble2Back = .15;
    public double wobble1LeanForward = 0.5, wobble2LeanForward = 0.5;
    public double hopper1Down = .2, hopper2Down = .8, hopper1Up = 0.565, hopper2Up = 0.435;
    public double angleAdjustLow = .5, angleAdjustHigh = .8;
    public double pusherIn = 0, pusherOut = 0.29;
    public double shotPower = .8, shotVelocity = 1500;
    public double shootHighVelocity = 1820, shootLowVelocity = 1500;
    public double shootHighPower = 1, shootLowPower = .875;
    public double intakePower = 1;

    //constants for flywheel pid
    public double velocityP = 0.0, velocityI = 0.0, velocityD = 0.0;
    public double integralActiveZone = 50;
}