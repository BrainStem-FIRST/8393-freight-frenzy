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
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(200);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 5;
    private boolean firstTimeRetract = true;
    private MovingStatistics stats = new MovingStatistics(10);
    private DecimalFormat tseDF = new DecimalFormat("###.##");

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this, color, true);
//        robot.pixyCam.start();

        robot.reset();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.turret.lock();
            robot.collector.tiltInit();
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
            telemetry.addData("Color Sensor Brightness", robot.collector.getBrightness());
            telemetry.addData("Is Freight Detected? ", robot.collector.isFreightCollectedColor());
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
            /*
            Keep running loop while:
            (color boolean is false or not past pose threshold) and not past second threshold
            Get out of loop when:
            (color boolean is true and past pose threshold) or past second threshold
             */
            boolean forward = true;
            while ((!robot.collector.isFreightCollectedColor() ||
                    robot.drive.getPoseEstimate().getX() <= coordinates.collectXMinThreshold()) &&
                        robot.drive.getPoseEstimate().getX() <= coordinates.collectXMaxThreshold()) {
                robot.drive.update();
                if (!robot.drive.isTrajectoryRunning()) {
                    if (forward) {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).forward(4).build());
                        forward = false;
                    } else {
                        robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(2).build());
                        forward = true;
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
            coordinates.updateCollect(robot.drive.getPoseEstimate().getX());

            TrajectorySequence depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .addTemporalMarker(0.65, 0, () -> {
                        waitForDeployCanceller.reset();
                        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                    })
                    .splineTo(coordinates.start().vec(), coordinates.depositTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(depositTrajectory);
            while(robot.drive.isTrajectoryRunning()) {
                robot.drive.update();

                if (firstTimeRetract && robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT
                        && robot.turret.isTurretZero() && !robot.collector.getRetractFull()) {
                    robot.collector.setGoal(Collector.Goal.RETRACTACTION);
                    firstTimeRetract = false;
                }
            }

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