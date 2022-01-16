package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.ArrayList;

public class BrainSTEMAutonomousCoordinates {
    /*
    Poses and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d startPos;
    private Pose2d startPosWarehouse = new Pose2d(6.5, 63, Math.toRadians(270));
    private Pose2d startPosCarousel = new Pose2d(-30.5, 63, Math.toRadians(270));

    private Pose2d shippingElementWaypoint;
    private Pose2d shippingElementCollect;
    private Pose2d shippingElementCollectFar = new Pose2d(5.7, 34, Math.toRadians(180));
    private Pose2d shippingElementCollectMiddle = new Pose2d(12, 49.5, Math.toRadians(90));
    private Pose2d shippingElementCollectNear = new Pose2d(1.5, 50, Math.toRadians(90));

    private Pose2d preloadDeposit;
    private Pose2d preloadDepositL1 = new Pose2d(1.5, 46, Math.toRadians(60)); //bottom
    private Pose2d preloadDepositL2 = new Pose2d(1.5, 47.5, Math.toRadians(60)); //middle
    private Pose2d preloadDepositL3 = new Pose2d(1.5, 44, Math.toRadians(60)); //top

    private Pose2d deposit;
    private Pose2d depositWarehouse = new Pose2d(0, 48, Math.toRadians(60));
    private Pose2d depositCarousel = new Pose2d(-28, 34, Math.toRadians(140));

    private Pose2d cycleWaypoint1 = new Pose2d(4, 60, Math.toRadians(0)); //x=34
    private Pose2d cycleWaypoint2 = new Pose2d(12, 65, Math.toRadians(0));
    private Pose2d cycleWaypoint3 = new Pose2d(30, 65, Math.toRadians(0));
    private Pose2d cycleCollect = new Pose2d(44, 65, Math.toRadians(0));
    private Pose2d cycleWaypoint1Reverse = new Pose2d(4, 62, Math.toRadians(0)); //x=34

    private Pose2d carouselDelivery = new Pose2d(-53, 55, Math.toRadians(0));
    private Pose2d carouselWait = new Pose2d(-36, 52, Math.toRadians(0));

    private Pose2d parkStart;
    private Pose2d parkWaypoint;
    private Pose2d parkEnd;

    private Pose2d parkWaypointWarehouse = new Pose2d(12, 37, Math.toRadians(0));
    private Pose2d parkWaypointCarousel = new Pose2d(5, 64.5, Math.toRadians(0));

    private Pose2d parkEndWarehouse = new Pose2d(55, 37, Math.toRadians(0));
    private Pose2d parkEndCarousel = new Pose2d(38, 64.5, Math.toRadians(0));

    private double shippingElementTangent = Math.toRadians(270);
    private double depositTangent;
    private double depositTangentWarehouse = Math.toRadians(250);
    private double depositTangentCarousel = Math.toRadians(140);

    private double cycleWaypoint1ForwardTangent = Math.toRadians(58);
    private double cycleForwardTangent = Math.toRadians(0);
    private double cycleReverseTangent = Math.toRadians(180);
    private double cycleWaypoint1ReverseTangent = Math.toRadians(225);

    private double carouselDeliveryTangent = Math.toRadians(135);

    private double parkTangent = Math.toRadians(0);

    private AllianceColor color;

    private Pose2d[] wallPositions;

    public BrainSTEMAutonomousCoordinates(AllianceColor color, StartLocation startLocation, BarcodePattern pattern) {
        this.color = color;
        if (startLocation == StartLocation.WAREHOUSE) {
            startPos = startPosWarehouse;
            deposit = depositWarehouse;
            parkStart = deposit;
            parkWaypoint = parkWaypointWarehouse;
            parkEnd = parkEndWarehouse;
            depositTangent = depositTangentWarehouse;
        } else {
            startPos = startPosCarousel;
            deposit = depositCarousel;
            parkStart = carouselWait;
            parkWaypoint = parkWaypointCarousel;
            parkEnd = parkEndCarousel;
            depositTangent = depositTangentCarousel;
        }
        shippingElementWaypoint = startPos.minus(new Pose2d(0,10, Math.toRadians(180)));

        switch(pattern) {
            case LEVELONE:
                preloadDeposit = preloadDepositL1;
                if(color == AllianceColor.BLUE) {
                    shippingElementCollect = shippingElementCollectFar;
                    shippingElementWaypoint = shippingElementWaypoint.plus(new Pose2d(0,0,Math.toRadians(90)));
                } else {
                    shippingElementCollect = shippingElementCollectNear;
                }
                break;
            case LEVELTWO:
                preloadDeposit = preloadDepositL2;
                shippingElementCollect = shippingElementCollectMiddle;
                break;
            case LEVELTHREE:
                preloadDeposit = preloadDepositL3;
                if(color == AllianceColor.RED) {
                    shippingElementCollect = shippingElementCollectFar;
                    shippingElementWaypoint = shippingElementWaypoint.plus(new Pose2d(0,0,Math.toRadians(90)));
                } else {
                    shippingElementCollect = shippingElementCollectNear;
                }
                break;
        }

        if (color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), flipHeading(startPos.getHeading()));
            shippingElementWaypoint = new Pose2d(shippingElementWaypoint.getX(), -shippingElementWaypoint.getY(), flipHeading(shippingElementWaypoint.getHeading()));
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), flipHeading(shippingElementCollect.getHeading()));
            preloadDeposit = new Pose2d(preloadDeposit.getX(), -preloadDeposit.getY(), flipHeading(preloadDeposit.getHeading()));
            deposit = new Pose2d(deposit.getX(), -deposit.getY(), flipHeading(deposit.getHeading()));
            cycleWaypoint1 = new Pose2d(cycleWaypoint1.getX(), -cycleWaypoint1.getY(), flipHeading(cycleWaypoint1.getHeading()));
            cycleWaypoint2 = new Pose2d(cycleWaypoint2.getX(), -cycleWaypoint2.getY(), flipHeading(cycleWaypoint2.getHeading()));
            cycleWaypoint3 = new Pose2d(cycleWaypoint3.getX(), -cycleWaypoint3.getY(), flipHeading(cycleWaypoint3.getHeading()));
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), flipHeading(cycleCollect.getHeading()));
            cycleWaypoint1Reverse = new Pose2d(cycleWaypoint1Reverse.getX(), -cycleWaypoint1Reverse.getY(), flipHeading(cycleWaypoint1Reverse.getHeading()));
            carouselDelivery = new Pose2d(carouselDelivery.getX(), -carouselDelivery.getY(), flipHeading(carouselDelivery.getHeading()));
            carouselWait = new Pose2d(carouselWait.getX(), -carouselWait.getY(), flipHeading(carouselWait.getHeading()));
            parkStart = new Pose2d(parkStart.getX(), -parkStart.getY(), flipHeading(parkStart.getHeading()));
            parkWaypoint = new Pose2d(parkWaypoint.getX(), -parkWaypoint.getY(), flipHeading(parkWaypoint.getHeading()));
            parkEnd = new Pose2d(parkEnd.getX(), -parkEnd.getY(), flipHeading(parkEnd.getHeading()));

            shippingElementTangent = flipHeading(shippingElementTangent);
            depositTangent = flipHeading(depositTangent);
            cycleWaypoint1ForwardTangent = flipHeading(cycleWaypoint1ForwardTangent);
            cycleForwardTangent = flipHeading(cycleForwardTangent);
            cycleReverseTangent = flipHeading(cycleReverseTangent);
            cycleWaypoint1ReverseTangent = flipHeading(cycleWaypoint1ReverseTangent);
            carouselDeliveryTangent = flipHeading(carouselDeliveryTangent);
            parkTangent = flipHeading(parkTangent);
        }
        wallPositions = new Pose2d[] {cycleWaypoint2, cycleWaypoint3, cycleCollect};
    }

    public Pose2d startPos() {
        return startPos;
    }

    public Pose2d shippingElementWaypoint() {
        return shippingElementWaypoint;
    }

    public Pose2d shippingElementCollect() {
        return shippingElementCollect;
    }

    public Pose2d preloadDeposit() {
        return preloadDeposit;
    }

    public Pose2d deposit() {
        return deposit;
    }

    public Pose2d cycleCollect() {
        return cycleCollect;
    }

    public void increaseCycleCollectPosition() {
        cycleCollect = cycleCollect.plus(new Pose2d(6, 0, 0));
    }

    public void increaseCycleCollectHeading() {
        cycleCollect = cycleCollect.plus(new Pose2d(0, 0,
                color == AllianceColor.RED ? Math.toRadians(10) : Math.toRadians(-10)));
    }

    public void increaseDeposit() {
        deposit = deposit.plus(new Pose2d(1, 0,
                color == AllianceColor.RED ? Math.toRadians(-5) : Math.toRadians(5)));
    }

    public void increaseWallPosition() {
        for (int i = 0; i < wallPositions.length; i++) {
            wallPositions[i] = wallPositions[i].plus(new Pose2d(0, Math.signum(wallPositions[i].getY()),0));
            Log.d("BSAutonomous", wallPositions[i].toString());
        }
    }

    public Pose2d cycleWaypoint1() {
        return cycleWaypoint1;
    }

    public Pose2d cycleWaypoint2() {
        return cycleWaypoint2;
    }

    public Pose2d cycleWaypoint3() {
        return cycleWaypoint3;
    }

    public Pose2d cycleWaypoint1Reverse() {
        return cycleWaypoint1Reverse;
    }

    public Pose2d carouselWait() {
        return carouselWait;
    }

    public Pose2d carouselDelivery() {
        return carouselDelivery;
    }

    public Pose2d parkStart() {
        return parkStart;
    }

    public double depositTangent() {
        return depositTangent;
    }

    public double cycleWaypoint1ForwardTangent() {
        return cycleWaypoint1ForwardTangent;
    }

    public double cycleWaypoint1ReverseTangent() {
        return cycleWaypoint1ReverseTangent;
    }

    public double cycleForwardTangent() {
        return cycleForwardTangent;
    }

    public double cycleReverseTangent() {
        return cycleReverseTangent;
    }

    public double carouselDeliveryTangent() {
        return carouselDeliveryTangent;
    }

    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
}