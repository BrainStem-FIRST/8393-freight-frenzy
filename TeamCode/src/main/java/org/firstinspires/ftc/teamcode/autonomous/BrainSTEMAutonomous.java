package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class BrainSTEMAutonomous extends LinearOpMode {
    private TimerCanceller waitForDeployCanceller = new TimerCanceller(2000); //2250
    private static final int WAIT_FOR_OPEN = 250;
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(750);
    private TimerCanceller waitForLiftAfterDriveCanceller = new TimerCanceller(300);
    private TimerCanceller waitForCollectorCanceller = new TimerCanceller(800);
    private TimerCanceller liftDefaultOffsetCanceller = new TimerCanceller(400);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 3;
    private boolean firstTimeRetract = true;
    private boolean depositDriveLoopCondition = true;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.pixie.start();
        robot.pixie.setColor(color);

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELONE);
        robot.collector.setAuto(true);

        robot.reset();
        robot.collector.tiltInit();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixie.teamShippingElementUpdate();
//            pattern = robot.pixie.tsePos();
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
            robot.update();
            telemetry.addLine("Ready for start");
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.addData("Team Shipping Element X", robot.pixie.tse_x);
            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

        robot.pixie.stop();

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
            while (!waitForRetractCanceller.isConditionMet()) {
                robot.update();
            }

            TrajectorySequence collectTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(collectTrajectory);
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
            while (robot.drive.isTrajectoryRunning()) {
                robot.update();
                robot.drive.update();
//                if (robot.collector.isFreightCollectedColor() &&
//                        robot.drive.getPoseEstimate().getX() > coordinates.collectXThreshold()) {
//                    //TODO: test endTrajectory()
//                    robot.drive.endTrajectory();
//                    robot.drive.setDrivePower(new Pose2d());
//                    telemetry.addLine("Ending early");
//                    telemetry.update();
//                    robot.drive.update();
//                    break;
//                }
            }
            robot.drive.waitForIdle();

            TrajectorySequence depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(coordinates.start().vec(), coordinates.depositTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(depositTrajectory);
            liftDefaultOffsetCanceller.reset();
            while(robot.drive.isTrajectoryRunning()) {
                robot.update();
                robot.drive.update();
                if (firstTimeRetract && robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT) {
                    robot.collector.setGoal(Collector.Goal.RETRACT);
                    firstTimeRetract = false;
                    waitForCollectorCanceller.reset();
                }
            }
            while(!waitForCollectorCanceller.isConditionMet());
            waitForDeployCanceller.reset();
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
            while(!waitForDeployCanceller.isConditionMet()) {
                robot.update();
            }
            robot.depositorLift.openPartial();
            coordinates.incrementCollect();
            //TODO: relocalize with COOL
        }

        sleep(WAIT_FOR_OPEN);
        waitForRetractCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
        while (!waitForRetractCanceller.isConditionMet()) {
            robot.update();
        }
        TrajectorySequence parkTrajectory = robot.drive.trajectorySequenceBuilder(coordinates.start())
                .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
                .build();

        robot.drive.followTrajectorySequence(parkTrajectory);
    }
}