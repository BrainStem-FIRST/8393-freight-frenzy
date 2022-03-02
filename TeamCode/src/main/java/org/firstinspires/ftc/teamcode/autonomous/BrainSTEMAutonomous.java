package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

import java.text.DecimalFormat;

public class BrainSTEMAutonomous extends LinearOpMode {
    private TimerCanceller waitForDeployCanceller = new TimerCanceller(1500);
    private static final int WAIT_FOR_OPEN = 300;
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(700);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 4;
    private boolean firstTimeRetract = true;
    private MovingStatistics stats = new MovingStatistics(10);
    private DecimalFormat tseDF = new DecimalFormat("###.##");

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this, color, true);
//        robot.pixyCam.start();

        robot.reset();
        robot.collector.tiltInit();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixyCam.teamShippingElementUpdate();
            stats.add(robot.pixyCam.tse_x);
            pattern = robot.pixyCam.tsePos(stats.getMean());
            switch(pattern) {
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
//            telemetry.addData("Health", robot.pixie.getHealth());
//            telemetry.addData("Connection", robot.pixie.getConnectionInfo());

            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

//        robot.pixyCam.stop();

        telemetry.clearAll();
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color);
        robot.drive.setPoseEstimate(coordinates.start());

        //deposit preload
        waitForDeployCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);

        while(!waitForDeployCanceller.isConditionMet()) {
            robot.update();
        }

        robot.depositorLift.openPartial();

        for (int i = 0; i < cycleTimes; i++) {
            firstTimeRetract = true;
            sleep(WAIT_FOR_OPEN);
            waitForRetractCanceller.reset();
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            while (!waitForRetractCanceller.isConditionMet()) {
                robot.update();
            }

            TrajectorySequence collectTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(collectTrajectory);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
            while (!robot.collector.isFreightCollected() ||
                    robot.drive.getPoseEstimate().getX() <= coordinates.collectXThreshold()) {
                robot.drive.update();
                if (!robot.drive.isTrajectoryRunning()) {
                    robot.drive.setDrivePower(new Pose2d(0.2,0,0));
                }

//                telemetry.addData("x RR", robot.drive.getPoseEstimate().getX());
//                telemetry.addData("y RR", robot.drive.getPoseEstimate().getY());
//                telemetry.addData("pose heading RR", robot.drive.getPoseEstimate().getHeading());
//
//                telemetry.addData("x COOL", robot.cool.getPoseEstimate().getX());
//                telemetry.addData("y COOL", robot.cool.getPoseEstimate().getY());
//                telemetry.addData("pose heading COOL", robot.cool.getPoseEstimate().getHeading());
            }
            //TODO: may need to move setGoal statement
            robot.drive.setDrivePower(new Pose2d());
            robot.collector.setGoal(Collector.Goal.RETRACT);
            robot.drive.endTrajectory();
            robot.drive.update();
            coordinates.updateCollect(robot.drive.getPoseEstimate().getX());

            TrajectorySequence depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(coordinates.start().vec(), coordinates.depositTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(depositTrajectory);
            while(robot.drive.isTrajectoryRunning()) {
                robot.drive.update();

                if (firstTimeRetract && robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT
                        && robot.turret.isTurretZero()) {
                    robot.collector.setGoal(Collector.Goal.RETRACTACTION);
                    firstTimeRetract = false;
                }
            }

            waitForDeployCanceller.reset();
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
            while(!waitForDeployCanceller.isConditionMet()) {
                robot.update();
            }
            robot.depositorLift.openPartial();
            //TODO: relocalize with COOL
//            robot.drive.setPoseEstimate(robot.cool.getPoseEstimate());
        }

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
        while(opModeIsActive()) {
            robot.drive.update();
        }
    }
}