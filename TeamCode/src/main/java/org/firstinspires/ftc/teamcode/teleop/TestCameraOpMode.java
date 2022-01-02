package org.firstinspires.ftc.teamcode.teleop;

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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public double xCorrection = 1;
    public double yCorrection = 1;
    public double headingCorrection = 1;

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    public double block_x;
    public double block_y;


    public double team_element_x;
    public double team_element_y;


    I2cDeviceSynch pixyCam;

    SampleMecanumDrive drive;


    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0, hardwareMap.appContext);
        }
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();

        pixyCam = hardwareMap.i2cDeviceSynch.get("Pixy Cam");


        pixyCam.engage();


        telemetry.addData("I2C Connection Info", pixyCam.getConnectionInfo());

        telemetry.update();





    }

    @Override
    public void loop() {

        //tele op movement
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();




        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();


        if (up == null) return;



        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        x = translation.getX() * xCorrection;
        y = translation.getY() * yCorrection;
        heading = up.pose.getHeading() * headingCorrection;

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = x + arrowX  / 2, y1 = y + arrowY / 2;
        double x2 = x + arrowX, y2 = y + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("CAMERA ON", slamra.isStarted());
        telemetry.addData("CONFIDENCE", up.confidence);
        telemetry.addData("X VALUE", x);
        telemetry.addData("Y VALUE", y);
        telemetry.addData("HEADING", heading);


        block_x = 0xff&pixyCam.read(0x50,5)[1];
        block_y = 0xff&pixyCam.read(0x50,5)[2];

        team_element_x = 0xff&pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&pixyCam.read(0x51,5)[2];

        telemetry.addData("PIXY ON", pixyCam.getHealthStatus());
        telemetry.addData("X value of block", block_x);
        telemetry.addData("X value of team element", team_element_x);


        telemetry.update();


    }

    @Override
    public void stop() {
        slamra.stop();
    }

}
