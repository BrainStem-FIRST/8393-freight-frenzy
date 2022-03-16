package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;

public class BrainSTEMAutonomousCoordinates {
    /*
    Poses and headings below defined as default for blue.
    Change to red occurs in update.
     */

    private Pose2d start = new Pose2d(15.5, 64.25, Math.toRadians(0)); //was 8 for alternate blue start

    private Pose2d collect = new Pose2d(43, start.getY(), Math.toRadians(0)); //x=43

    private Pose2d park;


    private double collectTangent = Math.toRadians(0);
    private double depositStartTangent = Math.toRadians(135);
    private double depositEndTangent = Math.toRadians(180);
    private double collectIncrement = 3;

    private double collectXMinThreshold = 38;
    private double collectXMaxThreshold = 60;
    private double depositX = 16;

    public BrainSTEMAutonomousCoordinates(AllianceColor color) {
        if (color == AllianceColor.RED) {
            start = new Pose2d(start.getX(), -start.getY(), flipHeading(start.getHeading()));
            collect = new Pose2d(collect.getX(), -collect.getY(), flipHeading(collect.getHeading()));
            depositStartTangent = flipHeading(depositStartTangent);
        }
        park = new Pose2d(collect.getX() + 5, collect.getY(), collect.getHeading());
    }

    public Pose2d start() {
        return start;
    }

    public Pose2d collect() {
        return collect;
    }

    public Pose2d deposit() {
        return new Pose2d(start.getX(), start.getY(), start.getHeading());
    }

    public Pose2d park() {
        return park;
    }

    public double collectTangent() {
        return collectTangent;
    }

    public double collectXMinThreshold() {
        return collectXMinThreshold;
    }

    public double collectXMaxThreshold() {
        return collectXMaxThreshold;
    }

    public double depositStartTangent() {
        return depositStartTangent;
    }
    public double depositEndTangent() {
        return depositEndTangent;
    }

    public void updateCollectX(double x) {
        collect = new Pose2d(x, collect.getY(), collect.getHeading());
    }

    public void shiftCollectYHeading(double y, double heading) {
        collect = new Pose2d(collect.getX(), collect.getY() + y, collect.getHeading() + heading);
    }

    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
}