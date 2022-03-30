package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

import java.text.DecimalFormat;

public class Autonomous extends LinearOpMode {
    private static final int WAIT_FOR_OPEN = 300;
    private static final int WAIT_FOR_CAROUSEL = 5500;

    private TimerCanceller waitForDeployCanceller = new TimerCanceller(1500);
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(400);
    private TimerCanceller waitToDeployCanceller = new TimerCanceller(650);

    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_THRESHOLD = 25.5;

    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;

    private int cycleTimes = 5;

    private boolean firstTimeRetract = true;
    private boolean firstTimeDeposit = true;
    private boolean endEarly = false;

    private BarcodePattern pattern = BarcodePattern.LEVELTHREE;

    private MovingStatistics tseXStats = new MovingStatistics(10);
    private DecimalFormat tseDF = new DecimalFormat("###.###");

    private BrainSTEMRobot robot;
    private Coordinates coordinates;

    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this, color, true);
//        robot.pixyCam.start();

        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.turret.lock();
            robot.collector.tiltInit();
            robot.pixyCam.teamShippingElementUpdate();
            tseXStats.add(robot.pixyCam.tse_x);
//            pattern = robot.pixyCam.tsePos(tseXStats.getMean());
            switch (pattern) {
                case LEVELONE:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELONE);
                    if (color == AllianceColor.BLUE && startLocation == StartLocation.WAREHOUSE) {
                        robot.turret.blueAutoOverride(true);
                    }
                    break;
                case LEVELTWO:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTWO);
                    break;
                case LEVELTHREE:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
                    break;
            }

//            if (gamepad1.x) robot.pixyCam.setThreshold(Direction.LEFT, robot.pixyCam.tse_x);
            if (gamepad1.y) robot.pixyCam.setThreshold(Direction.CENTER, robot.pixyCam.tse_x);
//            if (gamepad1.b) robot.pixyCam.setThreshold(Direction.RIGHT, robot.pixyCam.tse_x);

            robot.update();
            telemetry.addLine("Ready for start");
            telemetry.addData("Left Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.LEFT)));
            telemetry.addData("Gamepad 1 Y: Set Center Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.CENTER)));
            telemetry.addData("Right Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.RIGHT)));
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.addData("Current Team Shipping Element X", tseDF.format(robot.pixyCam.tse_x));
            telemetry.addData("Mean Team Shipping Element X", tseDF.format(tseXStats.getMean()));
            telemetry.addData("Color Sensor Brightness", robot.collector.getBrightness());
            telemetry.addData("Is Freight Detected? ", robot.collector.isFreightCollectedColor());
//            telemetry.addData("Health", robot.pixie.getHealth());
//            telemetry.addData("Connection", robot.pixie.getConnectionInfo());

            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }
        autoTime.reset();

//        robot.pixyCam.stop();

        telemetry.clearAll();
        telemetry.update();
        coordinates = new Coordinates(color, startLocation);
        robot.drive.setPoseEstimate(coordinates.start());

        if (startLocation == StartLocation.CAROUSEL) {
            robot.depositorLift.carouselAutoOverride(true);
            robot.turret.carouselAutoOverride(true);
            TrajectorySequence preloadTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate(), coordinates.carouselStartTangent())
                    .lineToLinearHeading(coordinates.deposit())
                    .build();
            robot.drive.followTrajectorySequence(preloadTrajectory);
            robot.turret.updateHeadingError(
                    robot.drive.getPoseEstimate().getHeading() - coordinates.deposit().getHeading());
        }

        depositSubsystems();
        switch(startLocation) {
            case WAREHOUSE:
                warehouse();
                break;
            case CAROUSEL:
                carousel();
                break;
        }
    }

    public void depositSubsystems() {
        waitForDeployCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);

        while (!waitForDeployCanceller.isConditionMet()) {
            robot.update();
        }

        robot.depositorLift.openPartial();

        robot.turret.blueAutoOverride(false);
    }

    public void retractSubsystems() {
        sleep(WAIT_FOR_OPEN);
        waitForRetractCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
        while (!waitForRetractCanceller.isConditionMet()) {
            robot.update();
        }
    }

    public void warehouse() {
        for (int i = 1; i <= cycleTimes; i++) {
            if (i == 3) {
                coordinates.shiftCollectYHeading();
            }
            firstTimeRetract = true;
            firstTimeDeposit = true;
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            retractSubsystems();

            TrajectorySequence collectTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .lineToSplineHeading(coordinates.collect())
                    .build();

            robot.drive.followTrajectorySequenceAsync(collectTrajectory);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);

            /*
            Keep running loop while:
            (color boolean is false or not past pose threshold) and not past second threshold
            Get out of loop when:
            (color boolean is true and past pose threshold) or past second threshold
             */
            boolean forward = true;
            int count = 0;
            while ((!robot.collector.isFreightCollectedColor() ||
                    robot.drive.getPoseEstimate().getX() <= coordinates.collectXMinThreshold()) &&
                    robot.drive.getPoseEstimate().getX() <= coordinates.collectXMaxThreshold()) {
                robot.drive.update();
                if (!robot.drive.isTrajectoryRunning()) {
                    if (count >= 2) {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(4).build());
                        count = 0;
                    } else if (forward) {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).forward(4).build());
                        forward = false;
                    } else {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(1).build());
                        forward = true;
                        count++;
                    }
                }

            }
            Log.d("BrainSTEM", "Time of collect number " + i + " : " + autoTime.seconds());
            if (autoTime.seconds() > TIME_THRESHOLD) {
                endEarly = true;
            }
            //TODO: may need to move setGoal statement
            robot.drive.setDrivePower(new Pose2d());
            if (robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT && robot.turret.isTurretZero()) {
                robot.collector.setRetractFull(true);
            } else {
                robot.collector.setRetractFull(false);
            }
            robot.collector.setGoal(Collector.Goal.RETRACT);
            robot.drive.endTrajectory();
            robot.drive.update();
            coordinates.updateCollectX(robot.drive.getPoseEstimate().getX());
            if (color == AllianceColor.BLUE && i <= 2) {
                robot.drive.setPoseEstimate(
                        robot.drive.getPoseEstimate().plus(new Pose2d(0,-0.5,0)));
            }

            if (endEarly) {
                break;
            }

            TrajectorySequence depositTrajectory;
            if (i >= 3) {
                depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate(), coordinates.depositStartTangent())
                        .splineToLinearHeading(coordinates.deposit(), coordinates.depositEndTangent())
                        .build();
            } else {
                depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .setReversed(true)
                        .lineToSplineHeading(coordinates.deposit())
                        .build();
            }

            waitToDeployCanceller.reset();
            robot.drive.followTrajectorySequenceAsync(depositTrajectory);
            while(robot.drive.isTrajectoryRunning()) {
                robot.drive.update();

                if (firstTimeRetract && robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT
                        && robot.turret.isTurretZero() && !robot.collector.getRetractFull()) {
                    robot.collector.setGoal(Collector.Goal.RETRACTACTION);
                    firstTimeRetract = false;
                }

                if (firstTimeDeposit && waitToDeployCanceller.isConditionMet() && robot.collector.getGoal() == Collector.Goal.DEFAULT) {
                    waitForDeployCanceller.reset();
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                    firstTimeDeposit = false;
                }
            }

            while(!waitForDeployCanceller.isConditionMet()) {
                robot.update();
            }
            robot.depositorLift.openPartial();
            //TODO: relocalize with COOL
//            robot.drive.setPoseEstimate(robot.cool.getPoseEstimate());
        }

        if (!endEarly) {
            retractSubsystems();

            TrajectorySequence parkTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .splineTo(coordinates.park().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(parkTrajectory);
            while (opModeIsActive()) {
                robot.drive.update();
            }
        }
    }

    public void carousel() {
        retractSubsystems();
        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);

        robot.collector.deploy();
        TrajectorySequence toCarouselTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .splineToLinearHeading(coordinates.spinCarousel(), coordinates.carouselEndTangent())
                .build();

        robot.drive.followTrajectorySequence(toCarouselTrajectory);

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(),
                        robot.drive.newVelocityConstraint(15, Math.toRadians(20)),
                robot.drive.newAccelerationConstraint(15))
                .forward(3).build());

        //spin carousel
        robot.carouselSpin.onAuto();
        sleep(WAIT_FOR_CAROUSEL);
        robot.carouselSpin.off();

        robot.collector.on();

        Trajectory collectTrajectory = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(),
                robot.drive.newVelocityConstraint(25, Math.toRadians(100)),
                robot.drive.newAccelerationConstraint(20))
                .lineToLinearHeading(coordinates.collectEndCarousel())
                .build();

        robot.drive.followTrajectory(collectTrajectory);

        robot.collector.setRetractFull(true);
        robot.collector.setGoal(Collector.Goal.RETRACT);

        TrajectorySequence depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .setReversed(true)
                .lineToLinearHeading(coordinates.deposit())
                .build();

        robot.drive.followTrajectorySequenceAsync(depositTrajectory);
        while(robot.collector.getGoal() != Collector.Goal.DEFAULT) {
            robot.update();
        }
        robot.drive.waitForIdle();

        depositSubsystems();
        retractSubsystems();

        TrajectorySequence parkTrajectory = robot.drive.trajectorySequenceBuilder
                (robot.drive.getPoseEstimate(), coordinates.carouselParkTangentStart())
                .splineToLinearHeading(coordinates.park(), coordinates.carouselEndTangent())
                .build();

        robot.drive.followTrajectorySequence(parkTrajectory);
        while(opModeIsActive()) {
            robot.collector.retract();
        }
    }
}