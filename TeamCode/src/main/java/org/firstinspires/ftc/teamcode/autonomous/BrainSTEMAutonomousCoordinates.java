package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BrainSTEMAutonomousCoordinates {
    /*
    Vectors and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d startPos;
    private Pose2d startPosWarehouse = new Pose2d(10, 63, Math.toRadians(90));
    private Pose2d startPosCarousel = new Pose2d(-36, 63, Math.toRadians(90));

    private Pose2d shippingElementCollect;
    private Pose2d shippingElementCollectOffset = new Pose2d(0, 11, Math.toRadians(0));

    private Pose2d deposit;
    private Pose2d depositWarehouse = new Pose2d(-2, 45, Math.toRadians(60));
    private Pose2d depositCarousel = new Pose2d(-28, 34, Math.toRadians(140));

    private Pose2d cycleWaypoint = new Pose2d(24, 64.5, Math.toRadians(0));
    private Pose2d cycleCollect = new Pose2d(50, 64.5, Math.toRadians(0));

    private Pose2d carouselWaypoint = new Pose2d(-25, 56, Math.toRadians(0));
    private Pose2d carouselDelivery = new Pose2d(-54, 56, Math.toRadians(0));

    private Pose2d parkPathStart;

    private Pose2d park;
    private Pose2d parkCarousel = new Pose2d(38, 60, Math.toRadians(0));

    private double depositTangent;
    private double depositTangentWarehouse = Math.toRadians(240);
    private double depositTangentCarousel = Math.toRadians(320);
    private double cycleParkTangent = Math.toRadians(0);

    private double carouselWaypointTangent = Math.toRadians(0);
    private double carouselDeliveryTangent = Math.toRadians(180);
    private double warehouseTurnAngle = Math.toRadians(-135);

    public BrainSTEMAutonomousCoordinates(AllianceColor color, StartLocation startLocation) {
        if (startLocation == StartLocation.WAREHOUSE) {
            startPos = startPosWarehouse;
            deposit = depositWarehouse;
            parkPathStart = deposit;
            park = cycleCollect;
            depositTangent = depositTangentWarehouse;
        } else {
            startPos = startPosCarousel;
            deposit = depositCarousel;
            parkPathStart = carouselDelivery;
            park = parkCarousel;
            depositTangent = depositTangentCarousel;
        }

        shippingElementCollect = startPos.minus(shippingElementCollectOffset);

        if(color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), startPos.getHeading());
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), shippingElementCollect.getHeading());
            deposit = new Pose2d(deposit.getX(), -deposit.getY(), deposit.getHeading());
            cycleWaypoint = new Pose2d(cycleWaypoint.getX(), -cycleWaypoint.getY(), cycleWaypoint.getHeading());
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), cycleCollect.getHeading());
            carouselWaypoint = new Pose2d(carouselWaypoint.getX(), -carouselWaypoint.getY(), carouselWaypoint.getHeading());
            carouselDelivery = new Pose2d(carouselDelivery.getX(), -carouselDelivery.getY(), carouselDelivery.getHeading());
            parkPathStart = new Pose2d(parkPathStart.getX(), -parkPathStart.getY(), parkPathStart.getHeading());
            park = new Pose2d(park.getX(), -park.getY(), park.getHeading());

            depositTangent = 2 * Math.PI - depositTangent;
            depositTangent = 2 * Math.PI - depositTangent;
            carouselWaypointTangent = 2 * Math.PI - carouselWaypointTangent;
            carouselDeliveryTangent = 2 * Math.PI - carouselDeliveryTangent;
            cycleParkTangent = 2 * Math.PI - cycleParkTangent;
        }
    }

    public Pose2d startPos() { return startPos; }
    public Pose2d shippingElementCollect() { return shippingElementCollect; }
    public Pose2d deposit() { return deposit; }

    public Pose2d carouselWaypoint() { return carouselWaypoint; }
    public Pose2d carouselDelivery() { return carouselDelivery; }

    public Pose2d cycleWaypoint() { return cycleWaypoint; }
    public Pose2d cycleCollect() { return cycleCollect; }

    public Pose2d parkPathStart() { return parkPathStart; }
    public Pose2d park() { return park; }

    public double depositTangent() { return depositTangent; }
    public double cycleParkTangent() { return cycleParkTangent; }

    public double carouselWaypointTangent() {
        return carouselWaypointTangent;
    }

    public double carouselDeliveryTangent() {
        return carouselWaypointTangent;
    }

    public double warehouseTurnAngle() {
        return warehouseTurnAngle;
    }
}