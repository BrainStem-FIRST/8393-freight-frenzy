package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

public class BrainSTEMAutonomous extends LinearOpMode {
    private static final int CYCLE_TIMES = 1;
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTHREE;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color, startLocation);

//        VuforiaLocalizer vuforia = initVuforia();
//        TFObjectDetector tfod = initTfod(vuforia);

        Trajectory startTrajectory = robot.drive.trajectoryBuilder(coordinates.startPos(), true)
                .lineTo(coordinates.shippingElementCollect().vec())
                .build();

        Trajectory depositTrajectory = robot.drive.trajectoryBuilder(coordinates.shippingElementCollect(), true)
                .addDisplacementMarker(0.5, 0, () -> {/*turn turret to proper angle*/})
                .splineTo(coordinates.deposit().vec(), coordinates.depositTangent())
                .build();

        TrajectorySequence warehouseSequence = robot.drive.trajectorySequenceBuilder(coordinates.shippingElementCollect())
                .addDisplacementMarker(0.25, 0, () -> {/*Turn on collector, reset turret, lift down, etc.*/})
                .splineTo(coordinates.cycleWaypoint().vec(), coordinates.cycleParkTangent())
                .lineTo(coordinates.cycleCollect().vec())
                .waitSeconds(2)
                .setReversed(true)
                .addDisplacementMarker(0.25, 0, () -> {/*Turn off collector, turn turret, lift up, etc.*/})
                .lineTo(coordinates.cycleWaypoint().vec())
                .splineTo(coordinates.deposit().vec(), coordinates.depositTangent())
                .waitSeconds(1.5)
                .build();

        TrajectorySequence carouselSequence = robot.drive.trajectorySequenceBuilder(coordinates.shippingElementCollect())
                .addDisplacementMarker(0.5, 0, () -> {/*turn turret to proper angle*/})
                .splineTo(coordinates.deposit().vec(), coordinates.depositTangent())
                .addDisplacementMarker(0.4, 0, () -> {/*Turn on carousel wheel, reset turret, lift down, etc.*/})
                .splineTo(coordinates.carouselWaypoint().vec(), coordinates.carouselWaypointTangent())
                .setReversed(true)
                .splineTo(coordinates.carouselDelivery().vec(), coordinates.carouselDeliveryTangent())
                .addDisplacementMarker(() -> {/*Deliver duck*/})
                .waitSeconds(5)
                .addDisplacementMarker(() -> {/*Turn off carousel wheel*/})
                .build();

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(coordinates.parkPathStart())
                .addDisplacementMarker(0.5, 0, () -> {/*Reset turret,  etc.*/})
                .splineTo(coordinates.cycleWaypoint().vec(), coordinates.cycleParkTangent())
                .splineTo(coordinates.park().vec(), coordinates.cycleParkTangent())
                .build();

//        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
//            if (tfod != null) {
//                List<Recognition> recognitions = tfod.getRecognitions();
//            }

            telemetry.addData("Status", "Waiting...");
//            telemetry.addData("IMU Calibrated during Loop?", robot.drive.getCalibrated());
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

//        if (tfod != null) {
//            tfod.shutdown();
//        }

        robot.drive.setPoseEstimate(coordinates.startPos());

        robot.drive.followTrajectory(startTrajectory);
        if (startLocation == StartLocation.WAREHOUSE) {
            robot.drive.turn(coordinates.warehouseTurnAngle());
            robot.drive.followTrajectory(depositTrajectory);
            for (int i = 0; i < CYCLE_TIMES; i++) {
                robot.drive.followTrajectorySequence(warehouseSequence);
            }
        } else {
            robot.drive.followTrajectory(depositTrajectory);
            robot.drive.followTrajectorySequence(carouselSequence);
        }
        robot.drive.followTrajectorySequence(parkSequence);
    }

    private VuforiaLocalizer initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AZ+MBZj/////AAABmW90kGIee0Slm9g+7us09sR8gfQg15reMkHgIiypD9aJ2DYw4XY8WrjdBLvMD1azS8YUemBDbUtT3WfG8UcZwxsNSAuu4dTx4XqyOgImF8o7zJQc5Xh9TO6Z1ctot2AcFwa4WKskl6ADSg/bux6eMDXJZKqgvlZXGWg5MV5dwc6UWJE4Xr1E0dfwudyxSoyHlHRYyXldTD9uD9/XPFqhww1ryPyztrn69nmRAx/jFnWUwwySjw4uG9Q+Icw7VDOqFQWbITKSOPiY64H5s4uMHpcNp9fPB0yYUeOMyn08ciZ0lqdRX/1N/J3oFS0yRSY7U98G+dTYe13OdYiPm7viAJXx8KhfowcbPuAu1ZOE39Tj";
        ;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "BarcodeCamera");

        //  Instantiate the Vuforia engine
        return ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private TFObjectDetector initTfod(VuforiaLocalizer vuforia) {
        String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("FreightFrenzy_BCDM.tflite", LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0/9.0);
        }

        return tfod;
    }
}