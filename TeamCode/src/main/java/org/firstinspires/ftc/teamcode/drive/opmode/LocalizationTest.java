package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    private static T265Camera slamra = null;

    public double x,y,heading;

    @Override
    public void runOpMode() throws InterruptedException {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0, hardwareMap.appContext);
        }

        slamra.stop();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slamra.start();

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();


            if (up == null) return;

            // We divide by 0.0254 to convert meters to inches
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            x = translation.getX();
            y = translation.getY() ;
            heading = up.pose.getHeading();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x_rr", poseEstimate.getX());
            telemetry.addData("y_rr", poseEstimate.getY());
            telemetry.addData("heading_rr", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("leftInches", drive.getWheelPositions().get(0));
            telemetry.addData("rightInches", drive.getWheelPositions().get(1));


            telemetry.addData("CAMERA ON", slamra.isStarted());
            telemetry.addData("CONFIDENCE", up.confidence);
            telemetry.addData("X VALUE_cam", x);
            telemetry.addData("Y VALUE_cam", y);
            telemetry.addData("HEADING_cam", Math.toDegrees(heading));

            telemetry.update();
        }
    }
}
