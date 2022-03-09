package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import java.util.Timer;

public class BrainSTEMAutonomous extends LinearOpMode {
    private TimerCanceller waitForDeployCanceller = new TimerCanceller(1500);
    private static final int WAIT_FOR_OPEN = 300;
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(200);
    private TimerCanceller waitToDeployCanceller = new TimerCanceller(650);
    private ElapsedTime autoTime = new ElapsedTime();
    private double TIME_THRESHOLD = 25.5;
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 5;
    private boolean firstTimeRetract = true;
    private boolean firstTimeDeposit = true;
    private boolean endEarly = false;
    private MovingStatistics stats = new MovingStatistics(10);
    private DecimalFormat tseDF = new DecimalFormat("###.###");

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this, color, true);
//        robot.pixyCam.start();

        robot.reset();
        if (color == AllianceColor.BLUE)
        while (!opModeIsActive() && !isStopRequested()) {
            robot.turret.lock();
            robot.collector.tiltInit();
            robot.pixyCam.teamShippingElementUpdate();
            stats.add(robot.pixyCam.tse_x);
            pattern = robot.pixyCam.tsePos(stats.getMean());
            switch (pattern) {
                case LEVELONE:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELONE);
                    break;
                case LEVELTWO:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTWO);
                    break;
                case LEVELTHREE:
                    robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
                    break;
            }

            if (gamepad1.x) robot.pixyCam.setThreshold(Direction.LEFT, robot.pixyCam.tse_x);
            if (gamepad1.y) robot.pixyCam.setThreshold(Direction.CENTER, robot.pixyCam.tse_x);
            if (gamepad1.b) robot.pixyCam.setThreshold(Direction.RIGHT, robot.pixyCam.tse_x);

            robot.update();
            telemetry.addLine("Ready for start");
            telemetry.addData("Gamepad 1 X: Set Left Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.LEFT)));
            telemetry.addData("Gamepad 1 Y: Set Center Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.CENTER)));
            telemetry.addData("Gamepad 1 B: Set Right Threshold", tseDF.format(robot.pixyCam.getThreshold(Direction.RIGHT)));
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.addData("Current Team Shipping Element X", tseDF.format(robot.pixyCam.tse_x));
            telemetry.addData("Mean Team Shipping Element X", tseDF.format(stats.getMean()));
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
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color);
        robot.drive.setPoseEstimate(coordinates.start());

        if (color == AllianceColor.BLUE) {
            robot.turret.blueAutoOverride(true);
            robot.depositorLift.blueAutoOverride(true);
        }

        //deposit preload
        waitForDeployCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);

        while(!waitForDeployCanceller.isConditionMet()) {
            robot.update();
        }

        robot.depositorLift.openPartial();

        for (int i = 1; i <= cycleTimes; i++) {
            if (i == 3) {
                coordinates.shiftCollectYHeading(color == AllianceColor.BLUE ? -5 : 5,
                        color == AllianceColor.BLUE ? Math.toRadians(-8) : Math.toRadians(8));
            }
            firstTimeRetract = true;
            firstTimeDeposit = true;
            sleep(WAIT_FOR_OPEN);
            waitForRetractCanceller.reset();
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            while (!waitForRetractCanceller.isConditionMet()) {
                robot.update();
            }

            TrajectorySequence collectTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .lineToSplineHeading(coordinates.collect())
                    .build();

            robot.drive.followTrajectorySequenceAsync(collectTrajectory);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
            robot.turret.blueAutoOverride(false);
            robot.depositorLift.blueAutoOverride(false);
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
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(2).build());
                        forward = true;
                        count++;
                    }
                }

//                telemetry.addData("x RR", robot.drive.getPoseEstimate().getX());
//                telemetry.addData("y RR", robot.drive.getPoseEstimate().getY());
//                telemetry.addData("pose heading RR", robot.drive.getPoseEstimate().getHeading());
//
//                telemetry.addData("x COOL", robot.cool.getPoseEstimate().getX());
//                telemetry.addData("y COOL", robot.cool.getPoseEstimate().getY());
//                telemetry.addData("pose heading COOL", robot.cool.getPoseEstimate().getHeading());
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
//            while (firstTimeDeposit) {
//                if (firstTimeDeposit && waitToDeployCanceller.isConditionMet() && robot.collector.getGoal() == Collector.Goal.DEFAULT) {
//                    waitForDeployCanceller.reset();
//                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
//                    firstTimeDeposit = false;
//                }
//            }

            while(!waitForDeployCanceller.isConditionMet()) {
                robot.update();
            }
            robot.depositorLift.openPartial();
            //TODO: relocalize with COOL
//            robot.drive.setPoseEstimate(robot.cool.getPoseEstimate());
        }

        if (!endEarly) {
            sleep(WAIT_FOR_OPEN);
            waitForRetractCanceller.reset();
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
            while (!waitForRetractCanceller.isConditionMet()) {
                robot.update();
            }

            TrajectorySequence parkTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.start())
                    .splineTo(coordinates.park().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(parkTrajectory);
            while (opModeIsActive()) {
                robot.drive.update();
            }
        }
    }
}