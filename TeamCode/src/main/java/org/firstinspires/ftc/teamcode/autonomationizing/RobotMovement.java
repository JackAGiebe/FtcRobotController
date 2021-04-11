package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

public class RobotMovement {

    private double xPower, yPower, turnPower;
    private double drive, strafe, rotate;

    OpMode op;
    RobotHardware robotHardware;
    MathFunctions mathFunctions = new MathFunctions();
    WorldPosition worldPosition;

    public RobotMovement(LinearOpMode op, RobotHardware robotHardware, WorldPosition worldPosition){
        this.op = op;
        this.robotHardware = robotHardware;
        this.worldPosition = worldPosition;
    }

    double distanceToPoint, xToPoint, yToPoint, angleToPreferred, moveSpeed;
    double movementXPower, movementYPower, movementTurnPower;
    double angleToPreferredMagnitude, angleToPreferredDirection;

    public void goToPoint(double x, double y, double angle, double p, double turnSpeed){

        angle = Math.toRadians(angle);

        distanceToPoint = Math.hypot(x - worldPosition.getxPosition(), y - worldPosition.getyPosition());

        p /= distanceToPoint > 4.5 ? 12 : 4;

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

    public void goToPoint(double x, double y, double angle){ goToPoint(x, y, angle, 1, 1); }

    public void goToPointMaxPower(double x, double y, double angle, double p, double turnSpeed, double maxPower){

        angle = Math.toRadians(angle);

        distanceToPoint = Math.hypot(x - worldPosition.getxPosition(), y - worldPosition.getyPosition());

        if(distanceToPoint > 3)
            p /= 12;
        else
            p /= 4;

        moveSpeed = Range.clip(distanceToPoint * p, 0, maxPower);

        xToPoint = x - worldPosition.getxPosition();
        yToPoint = y - worldPosition.getyPosition();

        movementXPower = Math.abs(xToPoint) > 1 ? xToPoint / Math.abs(Math.abs(xToPoint) + Math.abs(yToPoint)) : 0;
        movementYPower = Math.abs(yToPoint) > 1 ? yToPoint / Math.abs(Math.abs(yToPoint) + Math.abs(xToPoint)) : 0;

        xPower = movementXPower * moveSpeed;
        yPower = movementYPower * moveSpeed;

        angleAtPreferred(angle, turnSpeed);

        drive = Range.clip(yPower * Math.cos(worldPosition.getAngle()) + xPower * Math.sin(worldPosition.getAngle()), -maxPower, maxPower);
        strafe = -yPower * Math.sin(worldPosition.getAngle()) + xPower * Math.cos(worldPosition.getAngle());
    }

    public void goThroughPoint(double x, double y, double angle, double turnSpeed){ goToPoint(x, y, angle, Integer.MAX_VALUE, turnSpeed); }

    public void angleAtPreferred(double angle, double turnSpeed){
        angleToPreferred = Math.toDegrees(angle - worldPosition.getAngle());

        movementTurnPower = Range.clip((angleToPreferred/30), -1, 1);

        turnPower = movementTurnPower * turnSpeed;

        rotate = turnPower;
    }

    public void setMotorPowers(){
        robotHardware.frontLeft.setPower(drive - strafe + rotate);
        robotHardware.frontRight.setPower(drive - strafe - rotate);
        robotHardware.backLeft.setPower(drive + strafe + rotate);
        robotHardware.backRight.setPower(drive + strafe - rotate);
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
    public double getAngleToPreferred(){ return Math.abs(angleToPreferred); }
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
