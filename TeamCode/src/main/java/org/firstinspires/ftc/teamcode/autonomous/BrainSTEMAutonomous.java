package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class BrainSTEMAutonomous extends LinearOpMode {
    private TimerCanceller waitForDeployCanceller = new TimerCanceller(1550);
    private static final int WAIT_FOR_OPEN = 250;
    private TimerCanceller waitForRetractCanceller = new TimerCanceller(600);
    private TimerCanceller waitForLiftAfterDriveCanceller = new TimerCanceller(300);
    private TimerCanceller waitForCollectorCanceller = new TimerCanceller(200);
    private TimerCanceller waitToRetractCollectorCanceller = new TimerCanceller(400);
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 4;
    private boolean firstTimeRetract = true;
    private boolean depositDriveLoopCondition = true;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//        robot.pixie.start();
        robot.pixyCam.setColor(color);
        robot.turret.setColor(color);

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELONE);
        robot.collector.setAuto(true);
        robot.depositorLift.setAuto(true);
        robot.turret.setAuto(true);

        robot.reset();
        robot.collector.tiltInit();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixyCam.teamShippingElementUpdate();
            pattern = robot.pixyCam.tsePos();
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
            if (gamepad1.x) {
                robot.pixyCam.setThreshold(Direction.LEFT, robot.pixyCam.tse_x);
            }
            if (gamepad1.y) {
                robot.pixyCam.setThreshold(Direction.CENTER, robot.pixyCam.tse_x);
            }
            if (gamepad1.b) {
                robot.pixyCam.setThreshold(Direction.RIGHT, robot.pixyCam.tse_x);
            }
            robot.update();
            telemetry.addLine("Ready for start");
            telemetry.addData("Gamepad 1 X: Set Left Threshold", robot.pixyCam.getThreshold(Direction.LEFT));
            telemetry.addData("Gamepad 1 Y: Set Center Threshold", robot.pixyCam.getThreshold(Direction.CENTER));
            telemetry.addData("Gamepad 1 B: Set Right Threshold", robot.pixyCam.getThreshold(Direction.RIGHT));
            telemetry.addData("Barcode Pattern", pattern);
            telemetry.addData("Team Shipping Element X", robot.pixyCam.tse_x);
//            telemetry.addData("Health", robot.pixie.getHealth());
//            telemetry.addData("Connection", robot.pixie.getConnectionInfo());

            telemetry.update();
        }

        if (!opModeIsActive()) {
            return;
        }

//        robot.pixyCam.stop();
//        if (pattern == BarcodePattern.LEVELONE) {
//            cycleTimes --;
//        }

        telemetry.clearAll();
        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color);
        robot.drive.setPoseEstimate(coordinates.start());

        //deposit preload
        waitForDeployCanceller.reset();
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);

        while(!waitForDeployCanceller.isConditionMet()) {
            robot.update();
            Log.d("BrainSTEM", "lift target " + robot.depositorLift.getLiftTarget());
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
            while (robot.drive.isTrajectoryRunning()) {
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
            waitToRetractCollectorCanceller.reset();
            while(robot.drive.isTrajectoryRunning()) {
                robot.update();
                robot.drive.update();
                if (firstTimeRetract && waitToRetractCollectorCanceller.isConditionMet()
                        && robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.DEFAULT) {
                    robot.collector.setGoal(Collector.Goal.RETRACT);
                    firstTimeRetract = false;
                    waitForCollectorCanceller.reset();
                }
            }
//            while(!waitForCollectorCanceller.isConditionMet());
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
                .splineTo(coordinates.park().vec(), coordinates.collectTangent())
                .build();

        robot.drive.followTrajectorySequenceAsync(parkTrajectory);
        while(opModeIsActive()) {
            robot.drive.update();
        }
    }
}