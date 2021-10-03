package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BrainSTEMAutonomousCoordinates {
    /*
    Vectors and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d startPos;
    private Pose2d startPosWarehouse = new Pose2d(12, 64, Math.toRadians(90));
    private Pose2d startPosCarousel = new Pose2d(-36, 64, Math.toRadians(90));

    private Pose2d shippingElementCollect;
    private Pose2d shippingElementCollectOffset = new Pose2d(0, 16, Math.toRadians(0));

    private Pose2d deposit;
    private Pose2d depositWarehouse = new Pose2d(4, 34, Math.toRadians(30));
    private Pose2d depositCarousel = new Pose2d(-28, 34, Math.toRadians(140));

    private Pose2d cycleCollect = new Pose2d(44, 47, Math.toRadians(25));
    private Pose2d cycleDeposit = new Pose2d(7.5, 30, Math.toRadians(25));

    private Pose2d carouselWaypoint = new Pose2d(-25, 56, Math.toRadians(0));
    private Pose2d carouselDelivery = new Pose2d(-54, 56, Math.toRadians(0));

    private Pose2d parkPathStart;

    private Pose2d park;
    private Pose2d parkWarehouse = new Pose2d(44, 38, Math.toRadians(0));
    private Pose2d parkCarousel = new Pose2d(38, 60, Math.toRadians(0));

    private double preloadDepositTangent;
    private double preloadDepositTangentWarehouse = Math.toRadians(210);
    private double preloadDepositTangentCarousel = Math.toRadians(320);
    private double parkTangent = Math.toRadians(0);

    private double cycleCollectTangent = Math.toRadians(25);
    private double cycleDepositTangent = Math.toRadians(205);
    private double carouselWaypointTangent = Math.toRadians(0);
    private double carouselDeliveryTangent = Math.toRadians(180);

    public BrainSTEMAutonomousCoordinates(AllianceColor color, StartLocation startLocation) {
        if (startLocation == StartLocation.WAREHOUSE) {
            startPos = startPosWarehouse;
            deposit = depositWarehouse;
            parkPathStart = cycleDeposit;
            park = parkWarehouse;
            preloadDepositTangent = preloadDepositTangentWarehouse;
        } else {
            startPos = startPosCarousel;
            deposit = depositCarousel;
            parkPathStart = carouselDelivery;
            park = parkCarousel;
            preloadDepositTangent = preloadDepositTangentCarousel;
        }

        shippingElementCollect = startPos.minus(shippingElementCollectOffset);

        if(color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), startPos.getHeading());
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), shippingElementCollect.getHeading());
            deposit = new Pose2d(deposit.getX(), -deposit.getY(), deposit.getHeading());
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), cycleCollect.getHeading());
            cycleDeposit = new Pose2d(cycleDeposit.getX(), -cycleDeposit.getY(), cycleDeposit.getHeading());
            carouselWaypoint = new Pose2d(carouselWaypoint.getX(), -carouselWaypoint.getY(), carouselWaypoint.getHeading());
            carouselDelivery = new Pose2d(carouselDelivery.getX(), -carouselDelivery.getY(), carouselDelivery.getHeading());
            parkPathStart = new Pose2d(parkPathStart.getX(), -parkPathStart.getY(), parkPathStart.getHeading());
            park = new Pose2d(park.getX(), -park.getY(), park.getHeading());

            preloadDepositTangent = 2 * Math.PI - preloadDepositTangent;
            cycleCollectTangent = 2 * Math.PI - cycleCollectTangent;
            cycleDepositTangent = 2 * Math.PI - cycleDepositTangent;
            carouselWaypointTangent = 2 * Math.PI - carouselWaypointTangent;
            carouselDeliveryTangent = 2 * Math.PI - carouselDeliveryTangent;
            parkTangent = 2 * Math.PI - parkTangent;
        }
    }

    public Pose2d startPos() { return startPos; }
    public Pose2d shippingElementCollect() { return shippingElementCollect; }
    public Pose2d deposit() { return deposit; }

    public Pose2d carouselWaypoint() { return carouselWaypoint; }
    public Pose2d carouselDelivery() { return carouselDelivery; }

    public Pose2d cycleCollect() { return cycleCollect; }

    public Pose2d parkPathStart() { return parkPathStart; }
    public Pose2d park() { return park; }

    public double preloadDepositTangent() { return preloadDepositTangent; }
    public double parkTangent() { return parkTangent; }

    public double cycleCollectTangent() { return cycleCollectTangent; }
    public double cycleDepositTangent() { return cycleDepositTangent; }
    public double carouselWaypointTangent() {
        return carouselWaypointTangent;
    }
    public double carouselDeliveryTangent() {
        return carouselWaypointTangent;
    }
}