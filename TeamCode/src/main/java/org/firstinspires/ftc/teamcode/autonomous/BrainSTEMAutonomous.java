package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BrainSTEMAutonomous extends LinearOpMode {
    private static final int CYCLE_TIMES = 2;
    private double resetTheta = Math.toRadians(180);
    private double redSETheta = Math.toRadians(90);
    private double blueSETheta = Math.toRadians(270);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELONE;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.pixie.start();

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        robot.depositorLift.setAutoSE(true);

        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
//            robot.pixie.update();
//            pattern = robot.pixie.tsePos();
            robot.update();
            telemetry.addData("Status", "Waiting...");
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        robot.pixie.stop();

        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color, startLocation, pattern);

        robot.drive.setPoseEstimate(coordinates.startPos());

        TrajectorySequence seTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.startPos())
                .addDisplacementMarker(() -> {
                    if ((color == AllianceColor.BLUE && pattern == BarcodePattern.LEVELONE)
                            || (color == AllianceColor.RED && pattern == BarcodePattern.LEVELTHREE)) {
                        try {
                            robot.turret.resetTurret();
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                })
                .forward(7)
                .addDisplacementMarker(() -> {
                    if (pattern == BarcodePattern.LEVELTHREE && color == AllianceColor.RED) {
                        robot.turret.autoSpinTurret(redSETheta);
                    } else if (pattern == BarcodePattern.LEVELONE && color == AllianceColor.BLUE) {
                        robot.turret.autoSpinTurret(blueSETheta);
                    }
                })
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                })
                .lineTo(coordinates.shippingElementCollect().vec())
                .build();

        robot.drive.followTrajectorySequence(seTrajectory);
        while(opModeIsActive());

        if ((color == AllianceColor.BLUE && pattern == BarcodePattern.LEVELONE)
                || (color == AllianceColor.RED && pattern == BarcodePattern.LEVELTHREE)) {
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(4).build());
        } else {
            robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(2).build());
        }
        robot.depositorLift.clampSE();

        TrajectorySequence preloadTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    robot.depositorLift.setAutoSE(false);
                    switch(pattern) {
                        case LEVELONE:
                            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
                            break;
                        case LEVELTWO:
                            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.MIDDLE);
                            break;
                        case LEVELTHREE:
                            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                            break;
                    }
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    if (pattern != BarcodePattern.LEVELONE) {
                        robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP);
                    }
                })
                .lineToLinearHeading(coordinates.deposit())
                .addDisplacementMarker(() -> robot.depositorLift.open())
                .waitSeconds(0.2)
                .build();

        robot.drive.followTrajectorySequence(preloadTrajectory);

        TrajectorySequence warehouseSequence = robot.drive.trajectorySequenceBuilder(coordinates.deposit())
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleForwardWaypoint(), coordinates.cycleForwardTangent())
                .addDisplacementMarker(() -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                })
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())
                .waitSeconds(1)

                .setReversed(true)

                .addDisplacementMarker(() -> robot.collector.setGoal(Collector.Goal.RETRACT))
                .splineToSplineHeading(coordinates.cycleReverseWaypoint1(), coordinates.cycleReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .splineToSplineHeading(coordinates.cycleReverseWaypoint2(), coordinates.cycleReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP))
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .addDisplacementMarker(() -> robot.depositorLift.open())
                .waitSeconds(0.2)
                .build();

        for (int i = 0; i < CYCLE_TIMES; i++) {
            robot.drive.followTrajectorySequence(warehouseSequence);
        }

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(coordinates.deposit())
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN))
                .splineToSplineHeading(coordinates.cycleForwardWaypoint(), coordinates.cycleForwardTangent())
                .splineTo(coordinates.cycleCollect().vec(), coordinates.cycleForwardTangent())
                .build();

        robot.drive.followTrajectorySequence(parkSequence);
    }
}