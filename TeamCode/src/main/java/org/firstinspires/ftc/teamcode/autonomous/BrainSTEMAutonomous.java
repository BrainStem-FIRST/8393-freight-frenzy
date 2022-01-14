package org.firstinspires.ftc.teamcode.autonomous;

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
    private static final int CYCLE_TIMES = 1;
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
            robot.pixie.update();
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

        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color, startLocation, pattern);

        robot.drive.setPoseEstimate(coordinates.startPos());

        TrajectorySequence seTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.startPos())
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    robot.collector.setGoal(Collector.Goal.RETRACT);
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                })
                .turn(coordinates.startTurn())
                .addDisplacementMarker(() -> robot.depositorLift.releaseSE())
                .lineTo(coordinates.shippingElementCollect().vec())
                .build();

        robot.drive.followTrajectorySequence(seTrajectory);
        robot.depositorLift.extendSE();
        sleep(300);
        robot.depositorLift.clampSE();
        sleep(300);

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
                .lineToLinearHeading(coordinates.preloadDeposit())
                .build();

        robot.drive.followTrajectorySequence(preloadTrajectory);

        robot.depositorLift.openPartial();
//        sleep(200);

        TrajectorySequence warehouseSequence = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
                })
                .UNSTABLE_addDisplacementMarkerOffset(7, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ForwardTangent())
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleForwardTangent())
                .splineToSplineHeading(coordinates.cycleCollect(), coordinates.cycleForwardTangent())
                .setReversed(true)

                .UNSTABLE_addDisplacementMarkerOffset(1, () -> robot.collector.setGoal(Collector.Goal.RETRACT))
                .splineToSplineHeading(coordinates.cycleWaypoint2(), coordinates.cycleReverseTangent())

                .UNSTABLE_addDisplacementMarkerOffset(7, () -> robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTUP))
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleWaypoint1ReverseTangent())

                .addDisplacementMarker(() -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY))
                .splineToSplineHeading(coordinates.deposit(), coordinates.depositTangent())
                .build();

        for (int i = 0; i < CYCLE_TIMES; i++) {
            robot.drive.followTrajectorySequence(warehouseSequence);
            robot.depositorLift.openPartial();
//            sleep(200);
        }

        TrajectorySequence parkSequence = robot.drive.trajectorySequenceBuilder(coordinates.deposit())
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT))
                .UNSTABLE_addDisplacementMarkerOffset(7, () -> {
                    robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
                })
                .splineToSplineHeading(coordinates.cycleWaypoint1(), coordinates.cycleForwardTangent())
                .splineTo(coordinates.cycleCollect().vec(), coordinates.cycleForwardTangent())
                .build();

//        robot.drive.followTrajectorySequence(parkSequence);
    }
}