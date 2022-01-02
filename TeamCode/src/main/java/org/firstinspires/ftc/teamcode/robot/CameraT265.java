package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.spartronics4915.lib.T265Camera;

public class CameraT265 implements Component{

    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public double xCorrection = 1;
    public double yCorrection = 1;
    public double headingCorrection = 1;

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    public CameraT265(HardwareMap map)
    {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0, map.appContext);
        }

        slamra.stop();

        slamra.start();
    }


    @Override
    public void reset() {
        slamra.stop();
        slamra.start();
    }

    @Override
    public void update() {
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();


        if (up == null) return;



        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        x = translation.getX() * xCorrection;
        y = translation.getY() * yCorrection;
        heading = up.pose.getHeading() * headingCorrection;
    }

    @Override
    public String test() {
        return null;
    }

    public void setX (double new_x) {this.x = new_x; }

    public double getX() {return x;}

    public void setY (double new_y) {this.y = new_y; }

    public double getY() {return y;}

    public void setHeading (double new_heading) {this.heading = new_heading; }

    public double getHeading() {return heading;}
}
