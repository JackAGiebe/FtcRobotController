package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

public class RobotMovement {

    private double xPower, yPower, turnPower;
    private double drive, strafe, rotate;

    OpMode op;
    Telemetry telemetry;
    RobotHardware robotHardware;
    MathFunctions mathFunctions = new MathFunctions();
    WorldPosition worldPosition;

    public RobotMovement(LinearOpMode op, Telemetry telemetry, RobotHardware robotHardware, WorldPosition worldPosition){
        this.op = op;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        this.worldPosition = worldPosition;
    }

    double distanceToPoint, xToPoint, yToPoint, angleToPreferred, moveSpeed;
    double movementXPower, movementYPower, movementTurnPower;
    double angleToPreferredMagnitude, angleToPreferredDirection;

    public void goToPoint(double x, double y, double angle, double p, double turnSpeed){

        distanceToPoint = Math.hypot(x - worldPosition.getxPosition(), y - worldPosition.getyPosition());

        p /= 12;

        moveSpeed = Range.clip(distanceToPoint * p, 0, 1);

        xToPoint = x - worldPosition.getxPosition();
        yToPoint = y - worldPosition.getyPosition();

        movementXPower = Math.abs(xToPoint) > 1 ? xToPoint / Math.abs(Math.abs(xToPoint) + Math.abs(yToPoint)) : 0;
        movementYPower = Math.abs(yToPoint) > 1 ? yToPoint / Math.abs(Math.abs(yToPoint) + Math.abs(xToPoint)) : 0;

        xPower = movementXPower * moveSpeed;
        yPower = movementYPower * moveSpeed;

        angleAtPreferred(angle, turnSpeed);

        drive = yPower * Math.cos(worldPosition.getAngle()) + xPower * Math.sin(worldPosition.getAngle());
        strafe = -yPower * Math.sin(worldPosition.getAngle()) + xPower * Math.cos(worldPosition.getAngle());
    }

    //makes robot go to point
//    public void goToPointOld(double x, double y, double moveSpeed, double preferredHeading, double turnSpeed){
//
//        double distanceToPoint = Math.hypot(x - worldPosition.getxPosition(), y - worldPosition.getyPosition());
//
//        double absoluteAngleToPoint = Math.atan2(y-worldPosition.getyPosition(), x-worldPosition.getxPosition());
//        double relativeAngleToPoint = mathFunctions.angleWrap(absoluteAngleToPoint - Math.toRadians(worldPosition.getAngle()));
//
//        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToPoint;
//        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToPoint;
//        double relativeTurnAngle = relativeAngleToPoint + preferredHeading;
//
//        double movementXPower = relativeXToPoint / Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint);
//        double movementYPower = relativeYToPoint / Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint);
//        double movementTurnPower = Range.clip((relativeTurnAngle/30), -1, 1);
//
//        if(distanceToPoint < 10){
//            movementTurnPower = 0;
//        }
//
//        xPower = movementXPower * moveSpeed;
//        yPower = movementYPower * moveSpeed;
//        turnPower = movementTurnPower * turnSpeed;
//    }

    public void goThroughPoint(){

    }

    public void angleAtPoint(){}

    public void angleAtPreferred(double angle, double turnSpeed){

        angleToPreferredMagnitude = Math.abs(angle - Math.toDegrees(worldPosition.getAngle()));

        angleToPreferredDirection = (angle - Math.toDegrees(worldPosition.getAngle())) > 0 ? 1 : -1;

        angleToPreferred = angleToPreferredMagnitude * angleToPreferredDirection;

        movementTurnPower = Range.clip((angleToPreferred/30), -1, 1);

        movementTurnPower = Math.toDegrees(Math.abs(angleToPreferred)) < 1 ? 0 : movementTurnPower;

        turnPower = movementTurnPower * turnSpeed;

        rotate = turnPower;
    }

    public void setMotorPowers(){
        robotHardware.frontLeft.setPower(drive + strafe + rotate);
        robotHardware.frontRight.setPower(drive + strafe - rotate);
        robotHardware.backLeft.setPower(drive - strafe + rotate);
        robotHardware.backRight.setPower(drive - strafe - rotate);
    }

    public void setDrive(double drive){
        this.drive = drive;
    }

    public void setStrafe(double strafe){
        this.strafe = strafe;
    }

    public void setRotate(double rotate){
        this.rotate = rotate;
    }

    public double getyPower(){ return yPower; }
    public double getxPower(){ return xPower; }
    public double getTurnPower(){ return turnPower; }
    public double getDistanceToPoint(){ return distanceToPoint; }
    public double getxToPoint(){ return xToPoint; }
    public double getyToPoint(){ return yToPoint; }
    public double getAngleToPreferred(){ return angleToPreferred; }
    public double getMoveSpeed(){ return moveSpeed; }
    public double getMovementXPower(){ return movementXPower; }
    public double getMovementYPower(){ return movementYPower; }
    public double getMovementTurnPower(){ return movementTurnPower; }
    public double getDrive(){ return drive; }
    public double getStrafe(){ return strafe; }
    public double getRotate(){ return rotate; }
    public double getAngleToPreferredMagnitude(){ return angleToPreferredMagnitude; }
    public double getAngleToPreferredDirection(){ return angleToPreferredDirection; }
}
