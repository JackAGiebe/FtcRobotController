package org.firstinspires.ftc.teamcode.autonomationizing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.stuffs.Constants;
import org.firstinspires.ftc.teamcode.stuffs.MathFunctions;
import org.firstinspires.ftc.teamcode.stuffs.RobotHardware;

public class NewWorldPosition {

    RobotHardware robotHardware;
    Constants constants = new Constants();
    MathFunctions mathFunctions = new MathFunctions();
    LinearOpMode op;

    double leftCall = 0, rightCall = 0, normalCall = 0;
    private double left, right, normal;
    private double xPosition, yPosition, angle;
    private double previousRight = 0, previousLeft = 0, previousNormal = 0;
    public double theta, middle;

    //Constructor
    public NewWorldPosition(double xPosition, double yPosition, double angle, RobotHardware robotHardware, LinearOpMode op){
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.angle = Math.toRadians(mathFunctions.angleWrapDegrees(angle));
        this.robotHardware = robotHardware;
        this.op = op;
    }

    //Get calls from the opmode. The negative on the left call makes it so that forwards is positive for both right and left.
    public void giveEncoderHardwareCalls(double leftCall, double rightCall, double normalCall){
        this.leftCall = -leftCall;
        this.rightCall = rightCall;
        this.normalCall = normalCall;
    }

    //Updates the position using the hardware calls we just passed in.
    public void updateWorldPosition(){
        //Get changes in encoder position from the last call
        left = leftCall - previousLeft;
        right = rightCall - previousRight;
        normal = normalCall - previousNormal;

        //Update previous values
        previousLeft = leftCall;
        previousRight = rightCall;
        previousNormal = normalCall;

        //Get change in angle using difference between the 2 parallel encoders
        theta = ((left - right) / constants.countsPerInch) / constants.wheelBaseSeperation;

        //Calculate new angle using change in angle and put it through our angleWrap so that it goes from 2pi to 0;
        angle += theta;
        angle = mathFunctions.angleWrap(angle);

        //Adjust normal to account for the extra ticks that were counted due to turning
        normal -= theta * constants.normalDifferenceRadians;

        //How far the robot went straight forward (the right and left balance each other out so there aren't extra counts counted when turning)
        middle = (left + right) / 2;

        //Turn the middle and normal distances into inches
        middle /= constants.countsPerInch;
        normal /= constants.countsPerInch;

        //Update the x and y positions using the normal and middle based off what angle we're at
        yPosition += middle * Math.cos(angle) + normal * Math.sin(angle);
        xPosition += middle * Math.sin(angle) - normal * Math.cos(angle);
    }

    //Getters
    public double getLeft(){ return left; }
    public double getRight(){ return right; }
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