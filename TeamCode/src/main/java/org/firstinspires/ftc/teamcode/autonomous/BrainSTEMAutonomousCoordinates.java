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

    private Pose2d preloadDeposit = new Pose2d(4, 34, Math.toRadians(30));
    private Pose2d cycleCollect = new Pose2d(44, 47, Math.toRadians(25));
    private Pose2d cycleDeposit = new Pose2d(7.5, 30, Math.toRadians(25));

    private Pose2d park;
    private Pose2d parkWarehouse = new Pose2d(44, 38, Math.toRadians(0));
    private Pose2d parkCarousel = new Pose2d(39, 60, Math.toRadians(0));

    private double preloadDepositTangent = Math.toRadians(210);
    private double cycleCollectTangent = Math.toRadians(25);
    private double cycleDepositTangent = Math.toRadians(205);
    private double parkTangent = Math.toRadians(0);

    public void update(AllianceColor color, StartLocation startLocation) {
        if (startLocation == StartLocation.WAREHOUSE) {
            startPos = startPosWarehouse;
            park = parkWarehouse;
        } else {
            startPos = startPosCarousel;
            park = parkCarousel;
        }

        shippingElementCollect = startPos.minus(shippingElementCollectOffset);

        if(color == AllianceColor.RED) {
            startPos = new Pose2d(startPos.getX(), -startPos.getY(), startPos.getHeading());
            shippingElementCollect = new Pose2d(shippingElementCollect.getX(), -shippingElementCollect.getY(), shippingElementCollect.getHeading());
            preloadDeposit = new Pose2d(preloadDeposit.getX(), -preloadDeposit.getY(), preloadDeposit.getHeading());
            cycleCollect = new Pose2d(cycleCollect.getX(), -cycleCollect.getY(), cycleCollect.getHeading());
            cycleDeposit = new Pose2d(cycleDeposit.getX(), -cycleDeposit.getY(), cycleDeposit.getHeading());
            park = new Pose2d(park.getX(), -park.getY(), park.getHeading());

            preloadDepositTangent = 2 * Math.PI - preloadDepositTangent;
            cycleCollectTangent = 2 * Math.PI - cycleCollectTangent;
            cycleDepositTangent = 2 * Math.PI - cycleDepositTangent;
            parkTangent = 2 * Math.PI - parkTangent;
        }
    }

    public Pose2d startPos() { return startPos; }

    public Pose2d shippingElementCollect() { return shippingElementCollect; }

    public Pose2d preloadDeposit() { return preloadDeposit; }
    public Pose2d cycleCollect() { return cycleCollect; }
    public Pose2d cycleDeposit() { return cycleDeposit; }

    public Pose2d park() { return park; }

    public double preloadDepositTangent() { return preloadDepositTangent; }
    public double cycleCollectTangent() { return cycleCollectTangent; }
    public double cycleDepositTangent() { return cycleDepositTangent; }
    public double parkTangent() { return parkTangent; }
}