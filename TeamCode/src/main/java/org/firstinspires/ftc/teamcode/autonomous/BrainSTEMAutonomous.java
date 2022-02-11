package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class BrainSTEMAutonomous extends LinearOpMode {
    private TimerCanceller waitForDeployCanceller = new TimerCanceller(2250); //1800
    private static final int WAIT_FOR_OPEN = 200;
    private static final int WAIT_FOR_RETRACT = 350;
    protected AllianceColor color = AllianceColor.BLUE;
    protected StartLocation startLocation = StartLocation.WAREHOUSE;
    private BarcodePattern pattern = BarcodePattern.LEVELTWO;
    private int cycleTimes = 1;

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

        while (opModeIsActive()) {
            telemetry.addData("Lift height", robot.depositorLift.getHeight());
            telemetry.addData("Lift encoder", robot.depositorLift.getLiftPosition());
            telemetry.addData("Lift target", robot.depositorLift.getLiftTarget());
            telemetry.addData("Extension encoder", robot.depositorLift.getExtendPosition());
            telemetry.addData("Extension target", robot.depositorLift.getExtendTarget());
            telemetry.addData("Turret encoder", robot.turret.encoderPosition());
            telemetry.addData("Turret target", robot.turret.getTargetPosition());
            telemetry.update();
        }

        for (int i = 0; i < cycleTimes; i++) {
            sleep(WAIT_FOR_OPEN);
            robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
            sleep(WAIT_FOR_RETRACT);
            //deploy collector, retract depositor
            TrajectorySequence collectTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(collectTrajectory);
            robot.collector.setGoal(Collector.Goal.DEPLOY);
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
            while (robot.drive.isTrajectoryRunning()) {
                if (robot.collector.isFreightCollectedColor() &&
                        robot.drive.getPoseEstimate().getX() > coordinates.collectXThreshold()) {
                    //TODO: test endTrajectory()
                    robot.drive.endTrajectory();
                    robot.drive.setDrivePower(new Pose2d());
                    telemetry.addLine("Ending early");
                    telemetry.update();
                    robot.drive.update();
                    break;
                }
                robot.drive.update();
            }
            robot.drive.waitForIdle();

            //retract collector, deploy depositor
            TrajectorySequence depositTrajectory = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(coordinates.start().vec(), coordinates.depositTangent())
                    .build();

            robot.drive.followTrajectorySequenceAsync(depositTrajectory);
//            robot.collector.setGoal(Collector.Goal.RETRACT);
            while(robot.drive.isTrajectoryRunning()) {
//                if (robot.collector.getGoal() == Collector.Goal.DEFAULT) {
//                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
//                }
                telemetry.addData("Trajectory: ", robot.drive.getTrajectorySequenceString());
                telemetry.update();
            }
            robot.drive.waitForIdle();
            while(opModeIsActive());
            coordinates.incrementCollect();
            //TODO: relocalize with COOL
        }

//        sleep(WAIT_FOR_OPEN);
//        robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
//        sleep(WAIT_FOR_RETRACT);
//        Trajectory parkTrajectory = robot.drive.trajectoryBuilder(coordinates.start(),false)
//                .splineTo(coordinates.collect().vec(), coordinates.collectTangent())
//                .build();
//
//        robot.drive.followTrajectory(parkTrajectory);
    }
}