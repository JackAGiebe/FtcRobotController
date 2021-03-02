package org.firstinspires.ftc.teamcode.stuffs;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class MathFunctions {

    public double angleWrapDegrees(double angle){
        if(angle > 180){
            angle -= 360;
            if(angle > 180)
                angleWrap(angle);
        }
        else if(angle < -180){
            angle += 360;
            if(angle < -180)
                angleWrap(angle);
        }
        return angle;
    }

    public double angleWrap(double angle){
        if(angle > Math.PI){
            angle -= 2*Math.PI;
            if(angle > Math.PI)
                angleWrap(angle);
        }
        else if(angle < -Math.PI){
            angle += 2*Math.PI;
            if(angle < Math.PI)
                angleWrap(angle);
        }
        return angle;
    }

    public ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){

        //I don't really understand what the point of this is but steven did it in his tutorial so yeet
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = (int)(Math.round(linePoint2.y * .003));
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < .003){
            linePoint1.x = (int)(Math.round(linePoint2.x * .003));
        }

        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.y);

        double quadraticA = Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);

        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) * Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0*quadraticA*quadraticC)))/(2.0*quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            double minY = linePoint1.y < linePoint2.y ? linePoint1.y : linePoint2.y;
            double maxY = linePoint1.y > linePoint2.y ? linePoint1.y : linePoint2.y;

            int xRoot1Int = (int) (Math.round(xRoot1));
            int yRoot1Int = (int) (Math.round(yRoot1));

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1Int, yRoot1Int));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot2 = m1*(xRoot2 - x1) * y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            int xRoot2Int = (int) (Math.round(xRoot2));
            int yRoot2Int = (int) (Math.round(yRoot2));

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2Int, yRoot2Int));
            }
        }catch(Exception e){

        }
        return allPoints;
    }
}