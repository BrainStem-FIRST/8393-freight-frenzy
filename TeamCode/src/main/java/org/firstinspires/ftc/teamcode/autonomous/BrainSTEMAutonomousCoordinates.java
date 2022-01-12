package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

public class BrainSTEMAutonomousCoordinates {
    /*
    Poses and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d startPos;
    private Pose2d startPosWarehouse = new Pose2d(6.5, 63, Math.toRadians(270));
    private Pose2d startPosCarousel = new Pose2d(-30.5, 63, Math.toRadians(270));

    private Pose2d shippingElementCollect;
    private Pose2d shippingElementCollectFar = new Pose2d(8, 37, Math.toRadians(90));
    private Pose2d shippingElementCollectMiddle = new Pose2d(12, 47.5, Math.toRadians(90));
    private Pose2d shippingElementCollectNear = new Pose2d(3, 47.5, Math.toRadians(90));

    private Pose2d deposit;
    private Pose2d depositWarehouse = new Pose2d(-5.25, 46, Math.toRadians(70));
    private Pose2d depositCarousel = new Pose2d(-28, 34, Math.toRadians(140));

//    private Pose2d cycleWaypoint1 = new Pose2d(4, 50, Math.toRadians(70));
    private Pose2d cycleForwardWaypoint = new Pose2d(24, 65, Math.toRadians(0));
    private Pose2d cycleReverseWaypoint1 = new Pose2d(34, 66, Math.toRadians(0));
    private Pose2d cycleReverseWaypoint2 = new Pose2d(14, 66, Math.toRadians(0));
    private Pose2d cycleCollect = new Pose2d(50, 65, Math.toRadians(0));

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

    private double cycleWaypoint1Tangent = Math.toRadians(70);
    private double cycleForwardTangent = Math.toRadians(0);
    private double cycleReverseTangent = Math.toRadians(180);

    private double carouselDeliveryTangent = Math.toRadians(135);

    private double parkTangent = Math.toRadians(0);

    public BrainSTEMAutonomousCoordinates(AllianceColor color, StartLocation startLocation, BarcodePattern pattern) {
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

        if (pattern == BarcodePattern.LEVELTWO) {
            shippingElementCollect = shippingElementCollectMiddle;
        } else if ((color == AllianceColor.BLUE && pattern == BarcodePattern.LEVELONE)
                || (color == AllianceColor.RED && pattern == BarcodePattern.LEVELTHREE)) {
            shippingElementCollect = shippingElementCollectFar;
        } else {
            shippingElementCollect = shippingElementCollectNear;
        }

        if (color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), flipHeading(startPos.getHeading()));
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), flipHeading(shippingElementCollect.getHeading()));
            deposit = new Pose2d(deposit.getX(), -deposit.getY(), flipHeading(deposit.getHeading()));
            cycleForwardWaypoint = new Pose2d(cycleForwardWaypoint.getX(), -cycleForwardWaypoint.getY(), flipHeading(cycleForwardWaypoint.getHeading()));
            cycleReverseWaypoint1 = new Pose2d(cycleReverseWaypoint1.getX(), -cycleReverseWaypoint1.getY(), flipHeading(cycleReverseWaypoint1.getHeading()));
            cycleReverseWaypoint2 = new Pose2d(cycleReverseWaypoint2.getX(), -cycleReverseWaypoint2.getY(), flipHeading(cycleReverseWaypoint2.getHeading()));
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), flipHeading(cycleCollect.getHeading()));
            carouselDelivery = new Pose2d(carouselDelivery.getX(), -carouselDelivery.getY(), flipHeading(carouselDelivery.getHeading()));
            carouselWait = new Pose2d(carouselWait.getX(), -carouselWait.getY(), flipHeading(carouselWait.getHeading()));
            parkStart = new Pose2d(parkStart.getX(), -parkStart.getY(), flipHeading(parkStart.getHeading()));
            parkWaypoint = new Pose2d(parkWaypoint.getX(), -parkWaypoint.getY(), flipHeading(parkWaypoint.getHeading()));
            parkEnd = new Pose2d(parkEnd.getX(), -parkEnd.getY(), flipHeading(parkEnd.getHeading()));

            shippingElementTangent = flipHeading(shippingElementTangent);
            depositTangent = flipHeading(depositTangent);
            cycleWaypoint1Tangent = flipHeading(cycleWaypoint1Tangent);
            cycleForwardTangent = flipHeading(cycleForwardTangent);
            cycleReverseTangent = flipHeading(cycleReverseTangent);
            carouselDeliveryTangent = flipHeading(carouselDeliveryTangent);
            parkTangent = flipHeading(parkTangent);
        }
    }

    public Pose2d startPos() {
        return startPos;
    }

    public Pose2d shippingElementCollect() {
        return shippingElementCollect;
    }

    public Pose2d deposit() {
        return deposit;
    }

    public Pose2d cycleForwardWaypoint() {
        return cycleForwardWaypoint;
    }

    public Pose2d cycleCollect() {
        return cycleCollect;
    }

    public Pose2d cycleReverseWaypoint1() {
        return cycleReverseWaypoint1;
    }

    public Pose2d cycleReverseWaypoint2() {
        return cycleReverseWaypoint2;
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

    public Pose2d parkWaypoint() {
        return parkWaypoint;
    }

    public Pose2d parkEnd() {
        return parkEnd;
    }

    public double shippingElementTangent() {
        return shippingElementTangent;
    }

    public double depositTangent() {
        return depositTangent;
    }

    public double cycleWaypoint1Tangent() {
        return cycleWaypoint1Tangent;
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

    public double parkTangent() {
        return parkTangent;
    }

    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
}