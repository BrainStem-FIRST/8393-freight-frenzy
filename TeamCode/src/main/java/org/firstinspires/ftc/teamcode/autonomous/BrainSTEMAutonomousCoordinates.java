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

    private Pose2d start = new Pose2d(16.25, 64.25, Math.toRadians(0));

    private Pose2d collect = new Pose2d(43, start.getY(), Math.toRadians(0)); //x=43

    private double collectTangent = Math.toRadians(0);
    private double depositTangent = Math.toRadians(180);
    private double collectIncrement = 3;

    private double collectXThreshold = 38;

    public BrainSTEMAutonomousCoordinates(AllianceColor color) {
        if (color == AllianceColor.RED) {
            start = new Pose2d(start.getX(), -start.getY(), flipHeading(start.getHeading()));
            collect = new Pose2d(collect.getX(), -collect.getY(), flipHeading(collect.getHeading()));
        }
    }

    public Pose2d start() {
        return start;
    }

    public Pose2d collect() {
        return collect;
    }

    public double collectTangent() {
        return collectTangent;
    }

    public double collectXThreshold() {
        return collectXThreshold;
    }

    public double depositTangent() {
        return depositTangent;
    }

    public void incrementCollect() {
        collect = collect.plus(new Pose2d(collectIncrement,0,0));
    }

    private double flipHeading(double heading) { // radians
        return Angle.norm(2.0 * Math.PI - heading);
    }
}