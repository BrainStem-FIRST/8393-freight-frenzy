package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class COOLLocalizer implements Localizer {

    private static T265Camera slamra = null;


    public double xCorrection = 1;
    public double yCorrection = 1;
    public double headingCorrection = 1;

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    public COOLLocalizer (HardwareMap map) {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0, map.appContext);
        }

        //slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(23.75, 64.25), new Rotation2d(Math.toRadians(0))));
        slamra.stop();
        slamra.start();


    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();


        if (up == null) return null;



        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        x = translation.getX() * xCorrection;
        y = translation.getY() * yCorrection;
        heading = up.pose.getHeading() * headingCorrection;

        return new Pose2d(new Vector2d(x, y), heading);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(new Translation2d(pose2d.getX(), pose2d.getY()), new Rotation2d(pose2d.getHeading())));

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(new Vector2d(slamra.getLastReceivedCameraUpdate().velocity.vxMetersPerSecond, slamra.getLastReceivedCameraUpdate().velocity.vyMetersPerSecond), slamra.getLastReceivedCameraUpdate().velocity.omegaRadiansPerSecond);
    }

    @Override
    public void update() { }
}
