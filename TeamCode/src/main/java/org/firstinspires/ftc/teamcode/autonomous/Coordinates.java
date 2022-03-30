package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

public class Coordinates {
    /*
    Poses and headings below defined as default for blue.
    Change to red occurs in update.
     */

    //Overall variables
    private Pose2d start;
    private Pose2d deposit;
    private Pose2d park;

    //Warehouse variables
    private Pose2d startWarehouse = new Pose2d(15.5, 64.25, Math.toRadians(0));
    private Pose2d collect = new Pose2d(43, startWarehouse.getY(), Math.toRadians(0));

    private double collectXMinThreshold = 38;
    private double collectXMaxThreshold = 60;
    private double collectXAddFactor = 1.5;
    private double collectYShiftFactor = -5;
    private double collectHeadingShiftFactor = 2 * Math.PI - Math.toRadians(8);

    private double collectTangent = Math.toRadians(0);
    private double depositStartTangent = Math.toRadians(135);
    private double depositEndTangent = Math.toRadians(180);

    //Carousel variables
    private Pose2d startCarousel = new Pose2d(-31.5, 64.5, Math.toRadians(0));
    private Pose2d depositCarousel = new Pose2d(-34, 49, Math.toRadians(90));
    private Pose2d spinCarousel = new Pose2d(-59, 49, Math.toRadians(120));
    private Pose2d collectEndCarousel = new Pose2d(-36, 56.75, Math.toRadians(90));
    private Pose2d parkCarousel = new Pose2d(-62.5, 36, Math.toRadians(0));

    private double carouselStartTangent = Math.toRadians(270);
    private double carouselEndTangent = Math.toRadians(180);
    private double carouselParkTangentStart = Math.toRadians(90);

    public Coordinates(AllianceColor color, StartLocation location) {
        if (location == StartLocation.CAROUSEL) {
            start = startCarousel;
            deposit = depositCarousel;
            park = parkCarousel;
        } else {
            start = startWarehouse;
            deposit = start;
            park = new Pose2d(collect.getX() + 5, collect.getY(), collect.getHeading());
        }

        if (color == AllianceColor.RED) {
            //Overall
            start = flipPose(start);
            deposit = flipPose(deposit);
            park = flipPose(park);

            //Warehouse
            collect = flipPose(collect);
            depositStartTangent = flipHeading(depositStartTangent);
            collectYShiftFactor *= -1;
            collectHeadingShiftFactor = flipHeading(collectHeadingShiftFactor);

            //Carousel
            spinCarousel = flipPose(spinCarousel);
            collectEndCarousel = flipPose(collectEndCarousel);
            carouselStartTangent = flipHeading(carouselStartTangent);
            carouselParkTangentStart = flipHeading(carouselParkTangentStart);
        }
    }

    //Overall
    public Pose2d start() {
        return start;
    }
    public Pose2d deposit() {
        return deposit;
    }
    public Pose2d park() {
        return park;
    }

    //Warehouse
    public Pose2d collect() {
        return collect;
    }

    public double collectXMinThreshold() {
        return collectXMinThreshold;
    }
    public double collectXMaxThreshold() {
        return collectXMaxThreshold;
    }

    public double collectTangent() {
        return collectTangent;
    }
    public double depositStartTangent() {
        return depositStartTangent;
    }
    public double depositEndTangent() {
        return depositEndTangent;
    }

    public void updateCollectX(double x) {
        collect = new Pose2d(x + collectXAddFactor, collect.getY(), collect.getHeading());
    }
    public void shiftCollectYHeading() {
        collect = new Pose2d(collect.getX(),
                collect.getY() + collectYShiftFactor,
                collect.getHeading() + collectHeadingShiftFactor);
    }

    //Carousel
    public Pose2d spinCarousel() {
        return spinCarousel;
    }
    public Pose2d collectEndCarousel() {
        return collectEndCarousel;
    }

    public double carouselStartTangent() {
        return carouselStartTangent;
    }
    public double carouselEndTangent() {
        return carouselEndTangent;
    }
    public double carouselParkTangentStart() {
        return carouselParkTangentStart;
    }

    //Util
    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
    private Pose2d flipPose(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), flipHeading(pose.getHeading()));
    }
}