package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.ToggleButton;

import java.util.List;

@Autonomous
public class BrainSTEMAutonomous extends LinearOpMode {
    private static final int CYCLE_TIMES = 1;
    protected AllianceColor color = AllianceColor.BLUE;
    private BarcodePattern pattern = BarcodePattern.LEVELTHREE;
    private StartLocation startLocation = StartLocation.WAREHOUSE;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates();

        ToggleButton startButton = new ToggleButton();

        VuforiaLocalizer vuforia = initVuforia();
        TFObjectDetector tfod = initTfod(vuforia);

        while (!opModeIsActive() && !isStopRequested()) {
            if (tfod != null) {
                List<Recognition> recognitions = tfod.getRecognitions();
            }

            startButton.update(gamepad1.a);

            telemetry.addData("Status", "Waiting...");
//            telemetry.addData("IMU Calibrated during Loop?", robot.drive.getCalibrated());
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.addData("Gamepad A: Robot Start Location", startLocation);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        coordinates.update(color, startLocation);

        robot.drive.setPoseEstimate(coordinates.startPos());

        robot.start();

        Trajectory toShippingElementCollect = robot.drive.trajectoryBuilder()
                .lineTo(coordinates.shippingElementCollect().vec())
                .build();

        robot.drive.followTrajectory(toShippingElementCollect);

        /*
        Grab shipping element collect with "jewel arm"
         */

        Trajectory toPreloadDeposit = robot.drive.trajectoryBuilder()
                .addTemporalMarker(0.5, () -> {/*Turn turret*/})
                .splineTo(coordinates.preloadDeposit().vec(), coordinates.preloadDepositTangent())
                .build();

        robot.drive.followTrajectory(toShippingElementCollect);

        /*
        Deposit preload on proper level
         */

        for (int i = 0; i < CYCLE_TIMES; i++) {
            Trajectory toCycleCollect = robot.drive.trajectoryBuilder()
                    .addTemporalMarker(0.5, () -> {/*Turn on collector, reset turret, etc.*/})
                    .splineTo(coordinates.cycleCollect().vec(), coordinates.cycleCollectTangent())
                    .build();

            robot.drive.followTrajectory(toCycleCollect);

            /*
            Wait until collect confirmed?
             */

            Trajectory toCycleDeposit = robot.drive.trajectoryBuilder()
                    .addTemporalMarker(0.5, () -> {/*Turn off collector, turn turret, etc.*/})
                    .splineTo(coordinates.cycleDeposit().vec(), coordinates.cycleDepositTangent())
                    .build();

            robot.drive.followTrajectory(toCycleDeposit);
        }

        Trajectory toPark = robot.drive.trajectoryBuilder()
                .addTemporalMarker(0.5, () -> {/*Reset turret, etc.*/})
                .splineTo(coordinates.park().vec(), coordinates.parkTangent())
                .build();

        robot.drive.followTrajectory(toPark);
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