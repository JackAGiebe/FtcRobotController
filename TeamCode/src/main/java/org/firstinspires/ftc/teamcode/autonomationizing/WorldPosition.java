package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

public class WorldPosition {

    RobotHardware robotHardware;
    Constants constants = new Constants();
    MathFunctions mathFunctions = new MathFunctions();
    LinearOpMode op;

    double leftCall = 0, rightCall = 0, normalCall = 0;
    private double left, right, normal;
    private double xPosition, yPosition, angle;
    private double previousRight = 0, previousLeft = 0, previousNormal = 0;
    public double theta, thetaMagnitude, thetaDirection, previousAngle, middle;

    public WorldPosition(double xPosition, double yPosition, double angle, RobotHardware robotHardware, LinearOpMode op){
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.angle = Math.toRadians(mathFunctions.angleWrapDegrees(angle));
        this.robotHardware = robotHardware;
        this.op = op;
    }

    public void giveEncoderHardwareCalls(double leftCall, double rightCall, double normalCall){
        this.leftCall = -leftCall;
        this.rightCall = rightCall;
        this.normalCall = normalCall;
    }

    public void updateWorldPosition(){
        //Get changes in encoders
        left = leftCall - previousLeft;
        right = rightCall - previousRight;
        normal = normalCall - previousNormal;

        //Update previous values
        previousLeft = leftCall;
        previousRight = rightCall;
        previousNormal = normalCall;

        //Get change in angle
//        theta = ((left - right) / constants.wheelBaseSeperation) / constants.countsPerInch;
        //Calculate new angle
//        angle += theta;
        previousAngle = angle;
        angle = -Math.toRadians(robotHardware.imu.getAngularOrientation().firstAngle);
        thetaMagnitude = Math.abs(Math.abs(angle) - Math.abs(previousAngle));
        thetaDirection = angle - previousAngle > 0 ? 1 : -1;
        theta = thetaMagnitude * thetaDirection;

        //Adjust normal
        normal -= theta * constants.normalDifferenceRadians;

        //How far middle of left and right went (how far robot went forward)
        middle = (left + right) / 2;
//        middle = left;

        //Turn them into inches instead of ticks
        middle /= constants.countsPerInch;
        normal /= constants.countsPerInch;

        //Update the x and y positions
        yPosition += middle * Math.cos(angle) - normal * Math.sin(angle);
        xPosition += middle * Math.sin(angle) + normal * Math.cos(angle);
    }

    //Getters
    public double getxPosition(){ return xPosition; }
    public double getyPosition(){ return yPosition; }
    public double getAngle(){ return angle; }
    public double getAngleDegrees(){ return Math.toDegrees(angle); }

    //Setters
    public void setxPosition(double xPosition){ this.xPosition = xPosition; }
    public void setyPosition(double yPosition){ this.yPosition = yPosition; }
    public void setAngle(double angle){ this.angle = angle; }
    public void setAngleDegrees(double angle){ this.angle = Math.toRadians(angle); }
}