package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;

public class BrainSTEMAutonomous extends LinearOpMode {
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELONE;
    private int cycleTimes = 1;

    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.pixie.start();

        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        robot.collector.setAuto(true);

        robot.reset();
        robot.collector.tiltInit();
        while (!opModeIsActive() && !isStopRequested()) {
            robot.pixie.teamShippingElementUpdate();
            pattern = robot.pixie.tsePos();
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

        //deposit preload
        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);

        BrainSTEMAutonomousCoordinates coordinates = new BrainSTEMAutonomousCoordinates(color);
        robot.drive.setPoseEstimate(coordinates.start());

        sleep(800);

        robot.depositorLift.open();
        for (int i = 0; i < cycleTimes; i++) {
            //deploy collector, retract depositor
            Trajectory collectTrajectory = robot.drive.trajectoryBuilder(coordinates.start(),false)
                    .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectoryAsync(collectTrajectory);
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
            while (robot.drive.isBusy()) {
                if (robot.collector.isFreightCollectedColor() &&
                        robot.drive.getPoseEstimate().getX() > coordinates.collectXThreshold()) {
                    //TODO: test endTrajectory()
                    robot.drive.endTrajectory();
                }
            }
            robot.drive.waitForIdle();

            //retract collector, deploy depositor
            Trajectory depositTrajectory = robot.drive.trajectoryBuilder(coordinates.collect(), true)
                    .splineTo(coordinates.start().vec(), coordinates.depositTangent())
                    .build();

            robot.drive.followTrajectoryAsync(depositTrajectory);
            robot.collector.setGoal(Collector.Goal.RETRACT);
            while(robot.collector.getGoal() != Collector.Goal.DEFAULT);
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
            robot.drive.waitForIdle();
            coordinates.incrementCollect();
            //TODO: relocalize with COOL
        }
    }
}