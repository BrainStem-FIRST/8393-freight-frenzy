package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class BrainSTEMAutonomous extends LinearOpMode {
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELONE;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.pixie.start();

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        robot.depositorLift.setAutoSE(true);
        robot.collector.setAuto(true);

        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixie.teamShippingElementUpdate();
            pattern = robot.pixie.tsePos();
            robot.update();
            telemetry.addData("Status", "Waiting...");
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        robot.pixie.stop();
        Log.d("BSAutonomous", "Is pixy engaged? " + robot.pixie.isPixyEngaged());

        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color, startLocation, pattern);

        robot.drive.setPoseEstimate(coordinates.startPos());

        TrajectorySequence seTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.startPos())
                .addTemporalMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .lineToLinearHeading(coordinates.shippingElementWaypoint())
                .lineTo(coordinates.shippingElementCollect().vec())
                .build();

        robot.drive.followTrajectorySequenceAsync(seTrajectory);
        robot.depositorLift.releaseSE();
        robot.drive.waitForIdle();

        robot.depositorLift.extendSE();
        sleep(200);
        robot.depositorLift.clampSE();
        sleep(500);

        TrajectorySequence preloadTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(true)
                .addTemporalMarker(() -> {
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
                    if (pattern != BarcodePattern.LEVELONE) {
                        robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP);
                    }
                })
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    robot.depositorLift.setAutoSE(false);
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                })
                .waitSeconds(pattern == BarcodePattern.LEVELTHREE ? 0.4 : 0)
                .lineToLinearHeading(coordinates.preloadDeposit())
                .build();

        robot.drive.followTrajectorySequence(preloadTrajectory);

        robot.depositorLift.openPartial();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
        sleep(400);

        Log.d("BSAutonomous Cycle 1", coordinates.cycleCollect().toString());
        TrajectorySequence warehouseSequence1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())

                .setReversed(true)

                .UNSTABLE_addDisplacementMarkerOffset(3, () -> robot.collector.setGoal(Collector.Goal.RETRACT))
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleReverseTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleReverseTangent())

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP))
                .splineToSplineHeading(coordinates.cycleWaypoint1Reverse(), coordinates.cycleWaypoint1ReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .build();

        robot.drive.followTrajectorySequence(warehouseSequence1);
        robot.depositorLift.openPartial();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
        coordinates.increaseCycleCollectPosition();
        coordinates.increaseDeposit();
        coordinates.increaseWallPosition();
        sleep(400);

        Log.d("BSAutonomous Cycle 2", coordinates.cycleCollect().toString());
        TrajectorySequence warehouseSequence2 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())

                .setReversed(true)

                .UNSTABLE_addDisplacementMarkerOffset(3, () -> robot.collector.setGoal(Collector.Goal.RETRACT))
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleReverseTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleReverseTangent())

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP))
                .splineToSplineHeading(coordinates.cycleWaypoint1Reverse(), coordinates.cycleWaypoint1ReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .build();

        robot.drive.followTrajectorySequence(warehouseSequence2);
        robot.depositorLift.openPartial();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
//        coordinates.increaseCycleCollectPosition();
        coordinates.increaseCycleCollectHeading();
        coordinates.increaseDeposit();
        coordinates.increaseWallPosition();
        sleep(400);

        Log.d("BSAutonomous Cycle 3", coordinates.cycleCollect().toString());
        TrajectorySequence warehouseSequence3 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())

                .setReversed(true)

                .UNSTABLE_addDisplacementMarkerOffset(3, () -> robot.collector.setGoal(Collector.Goal.RETRACT))
                .splineToSplineHeading(coordinates.cycleWaypoint3(), coordinates.cycleReverseTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleReverseTangent())

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP))
                .splineToSplineHeading(coordinates.cycleWaypoint1Reverse(), coordinates.cycleWaypoint1ReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .build();

        robot.drive.followTrajectorySequence(warehouseSequence3);
        robot.depositorLift.openPartial();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
        sleep(400);

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(coordinates.deposit())
                .UNSTABLE_addDisplacementMarkerOffset(7, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN))
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())
                .build();

        robot.drive.followTrajectorySequence(parkSequence);
    }
}