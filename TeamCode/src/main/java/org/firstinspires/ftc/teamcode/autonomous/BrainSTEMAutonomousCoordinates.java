package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BrainSTEMAutonomousCoordinates {
    /*
    Vectors and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d startPos;
    private Pose2d startPosWarehouse = new Pose2d(12, 63, Math.toRadians(90));
    private Pose2d startPosCarousel = new Pose2d(-36, 63, Math.toRadians(90));

    private Pose2d shippingElementCollect;
    private Pose2d shippingElementCollectOffset = new Pose2d(0, 17, Math.toRadians(0));

    private Pose2d deposit;
    private Pose2d depositWarehouse = new Pose2d(1, 40, Math.toRadians(70));
    private Pose2d depositCarousel = new Pose2d(-28, 34, Math.toRadians(140));

    private Pose2d cycleWaypoint1 = new Pose2d(4, 50, Math.toRadians(70));
    private Pose2d cycleWaypoint2 = new Pose2d(24, 64.5, Math.toRadians(0));
    private Pose2d cycleCollect = new Pose2d(50, 64.5, Math.toRadians(0));

    private Pose2d carouselDelivery = new Pose2d(-53, 55, Math.toRadians(0));
    private Pose2d carouselWait = new Pose2d(-36, 52, Math.toRadians(0));

    private Pose2d parkStart;
    private Pose2d parkWaypoint;
    private Pose2d parkEnd;

    private Pose2d parkWaypointWarehouse = new Pose2d(12, 37, Math.toRadians(0));
    private Pose2d parkWaypointCarousel = new Pose2d(5, 64.5, Math.toRadians(0));

    private Pose2d parkEndWarehouse = new Pose2d(55, 37, Math.toRadians(0));
    private Pose2d parkEndCarousel = new Pose2d(38, 64.5, Math.toRadians(0));

    private double depositTangent;
    private double depositTangentWarehouse = Math.toRadians(250);
    private double depositTangentCarousel = Math.toRadians(140);

    private double cycleWaypoint1Tangent = Math.toRadians(70);
    private double cycleForwardTangent = Math.toRadians(0);
    private double cycleReverseTangent = Math.toRadians(180);

    private double carouselDeliveryTangent = Math.toRadians(135);

    private double parkTangent = Math.toRadians(0);

    public BrainSTEMAutonomousCoordinates(AllianceColor color, StartLocation startLocation) {
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

        shippingElementCollect = startPos.minus(shippingElementCollectOffset);

        if(color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), startPos.getHeading());
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), shippingElementCollect.getHeading());
            deposit = new Pose2d(deposit.getX(), -deposit.getY(), deposit.getHeading());
            cycleWaypoint1 = new Pose2d(cycleWaypoint1.getX(), -cycleWaypoint1.getY(), cycleWaypoint1.getHeading());
            cycleWaypoint2 = new Pose2d(cycleWaypoint2.getX(), -cycleWaypoint2.getY(), cycleWaypoint2.getHeading());
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), cycleCollect.getHeading());
            carouselDelivery = new Pose2d(carouselDelivery.getX(), -carouselDelivery.getY(), carouselDelivery.getHeading());
            carouselWait = new Pose2d(carouselWait.getX(), -carouselWait.getY(), carouselWait.getHeading());
            parkStart = new Pose2d(parkStart.getX(), -parkStart.getY(), parkStart.getHeading());
            parkWaypoint = new Pose2d(parkWaypoint.getX(), -parkWaypoint.getY(), parkWaypoint.getHeading());
            parkEnd = new Pose2d(parkEnd.getX(), -parkEnd.getY(), parkEnd.getHeading());

            depositTangent = 2 * Math.PI - depositTangent;
            cycleWaypoint1Tangent = 2 * Math.PI  - cycleWaypoint1Tangent;
            cycleForwardTangent = 2 * Math.PI - cycleForwardTangent;
            cycleReverseTangent = 2 * Math.PI - cycleReverseTangent;
            carouselDeliveryTangent = 2 * Math.PI - carouselDeliveryTangent;
            parkTangent = 2 * Math.PI - parkTangent;
        }
    }

    public Pose2d startPos() { return startPos; }
    public Pose2d shippingElementCollect() { return shippingElementCollect; }
    public Pose2d deposit() { return deposit; }

    public Pose2d cycleWaypoint1() { return cycleWaypoint1; }
    public Pose2d cycleWaypoint2() { return cycleWaypoint2; }
    public Pose2d cycleCollect() { return cycleCollect; }

    public Pose2d carouselWait() {
        return carouselWait;
    }
    public Pose2d carouselDelivery() { return carouselDelivery; }

    public Pose2d parkStart() { return parkStart; }
    public Pose2d parkWaypoint() { return parkWaypoint; }
    public Pose2d parkEnd() { return parkEnd; }

    public double depositTangent() { return depositTangent; }
    public double cycleWaypoint1Tangent() { return cycleWaypoint1Tangent; }
    public double cycleForwardTangent() { return cycleForwardTangent; }
    public double cycleReverseTangent() { return cycleReverseTangent; }
    public double carouselDeliveryTangent() {
        return carouselDeliveryTangent;
    }
    public double parkTangent() { return parkTangent; }
}
