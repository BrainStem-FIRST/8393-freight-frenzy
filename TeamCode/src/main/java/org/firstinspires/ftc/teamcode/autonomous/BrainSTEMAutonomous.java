package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BrainSTEMAutonomous extends LinearOpMode {
    private static final int CYCLE_TIMES = 1;
    private double shippingElementTheta;
    private double shippingElementThetaLeft = Math.toRadians(200);
    private double shippingElementThetaCenter = Math.toRadians(180);
    private double shippingElementThetaRight = Math.toRadians(160);
    private double depositTheta;
    private double depositThetaWarehouse = Math.toRadians(160);
    private double depositThetaCarousel = Math.toRadians(200);
    private double resetTheta = Math.toRadians(180);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTHREE;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color, startLocation);
        robot.pixie.start();

        depositTheta = startLocation == StartLocation.WAREHOUSE ? depositThetaWarehouse : depositThetaCarousel;

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);

        TrajectorySequence seTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.startPos())
                .forward(7)
                .turn(Math.toRadians(180))
                .splineToSplineHeading(coordinates.shippingElementCollect(), coordinates.shippingElementTangent())
                .build();

        Trajectory preloadTrajectory = robot.drive.trajectoryBuilder(coordinates.shippingElementCollect(), true)
                .splineToLinearHeading(coordinates.deposit(), coordinates.depositTangent())
                .build();

        TrajectorySequence warehouseSequence = robot.drive.trajectorySequenceBuilder(coordinates.shippingElementCollect())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1Tangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.turret.autoSpinTurret(resetTheta);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                })
                .splineTo(coordinates.cycleCollect().vec(), coordinates.cycleForwardTangent())
                .waitSeconds(2)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.collector.setGoal(Collector.Goal.RETRACT);
                })
                .splineTo(coordinates.cycleWaypoint2().vec(), coordinates.cycleReverseTangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.depositTangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP);
                    robot.turret.autoSpinTurret(depositTheta);
                })
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .addDisplacementMarker(() -> {robot.depositorLift.open();})
                .waitSeconds(1.5)
                .build();

//        TrajectorySequence carouselSequence = robot.drive.trajectorySequenceBuilder(coordinates.shippingElementCollect())
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//                    robot.depositorLift.setGoal(DepositorLift.Goal.RETRACT);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    robot.turret.autoSpinTurret(resetTheta);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    robot.depositorLift.autoLiftDown();
//                })
//                .splineToSplineHeading(coordinates.carouselDelivery(), coordinates.carouselDeliveryTangent())
//                .addDisplacementMarker(() -> {robot.carouselSpin.autonomousSpinCarousel(color);})
//                .forward(1)
//                .waitSeconds(4)
//                .back(1)
//                .addDisplacementMarker(() -> {robot.carouselSpin.stopCarousel();})
//                .setReversed(true)
//                .splineToSplineHeading(coordinates.carouselWait(), coordinates.parkTangent())
//                .build();

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(coordinates.parkStart(), coordinates.parkTangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1Tangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.turret.autoSpinTurret(resetTheta);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                })
                .splineTo(coordinates.cycleCollect().vec(), coordinates.cycleForwardTangent())
                .build();

//        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixie.update();
            telemetry.addData("Status", "Waiting...");
            telemetry.addData("Marker x", robot.pixie.team_element_x);
            telemetry.addData("Marker y", robot.pixie.team_element_y);
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        robot.pixie.stop();

        switch(pattern) {
            case LEVELONE:
                shippingElementTheta = shippingElementThetaLeft;
                break;
            case LEVELTWO:
                shippingElementTheta = shippingElementThetaCenter;
                break;
            case LEVELTHREE:
                shippingElementTheta = shippingElementThetaRight;
                break;
        }

        robot.drive.setPoseEstimate(coordinates.startPos());


        robot.drive.followTrajectorySequenceAsync(seTrajectory);
//        robot.depositorLift.setGoal(DepositorLift.Goal.DEPLOY);
        sleep(500);
//        robot.turret.autoSpinTurret(shippingElementTheta);
        robot.drive.waitForIdle();
//        robot.depositorLift.clampSE();
//        sleep(1000);

        robot.drive.followTrajectoryAsync(preloadTrajectory);
            //needs to be here bc shipping element has to be at L1
//        switch(pattern) {
//            case LEVELONE:
//                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
//                break;
//            case LEVELTWO:
//                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.MIDDLE);
//                break;
//            case LEVELTHREE:
//                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
//                break;
//        }
//        if (pattern != BarcodePattern.LEVELONE) {
//            robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP);
//        }
//        robot.turret.autoSpinTurret(depositThetaWarehouse);
        robot.drive.waitForIdle();
        while(opModeIsActive());
        robot.depositorLift.open();

//        if (startLocation == StartLocation.WAREHOUSE) {
            for (int i = 0; i < CYCLE_TIMES; i++) {
                robot.drive.followTrajectorySequence(warehouseSequence);
            }
//        } else {
//            robot.drive.followTrajectorySequence(carouselSequence);
//        }
        robot.drive.followTrajectorySequence(parkSequence);
    }
}