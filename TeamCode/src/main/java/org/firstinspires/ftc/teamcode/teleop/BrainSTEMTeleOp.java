package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.robot.Direction;

public class BrainSTEMTeleOp extends LinearOpMode {
    //Initializes joystick storage variables'
    private double leftStickX, leftStickY, rightStickX;

    private static final int EXTEND_ADJUST_INTERVAL = 80;
    private static final int TURRET_ADJUST_INTERVAL = 15;
    //driving tuning factor
    private double leftYTuning = 1;
    //strafing tuning factor
    private double leftXTuning = 1;
    //turning tuning factor
    private double rightXTuning = 1;

    //Variables for polar coordinate driving
    private double r;
    private double theta;
    private double turning;

    private double driveInterpolationFactor = 3;

    private ToggleButton collectOnButton = new ToggleButton();
    private ToggleButton capModeButton = new ToggleButton();

    private StickyButton depositButton = new StickyButton();
    private StickyButton carouselButton = new StickyButton();
    private StickyButton extendAdjustOutButton = new StickyButton();
    private StickyButton extendAdjustInButton = new StickyButton();
    private StickyButton turretAdjustLeftButton = new StickyButton();
    private StickyButton turretAdjustRightButton = new StickyButton();

    private Driver1 driver1 = new Driver1();
    private Driver2 driver2 = new Driver2();

    private BrainSTEMRobot robot;

    protected AllianceColor color = null;

    private boolean extended = false;
    private boolean deployFirstTime = false, retractFirstTime = false;

    private class Driver1 {
        private boolean collectOn;
        private boolean gateOverride;
        private boolean reverseCollect;
        private boolean retract;
        private boolean liftUpCap, liftDownCap;
        private boolean extendOutCap, extendRetractCap;
    }

    private class Driver2 {
        private boolean depositHigh, depositMid, depositLow;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this);
        robot.reset();
        robot.depositorLift.setCap(false);
        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        robot.depositorLift.rotateCollect();
//        robot.turret.resetTurret();

        while(opModeIsActive()) {
            robot.update();
            mapControls(driver1, driver2);
            runLoop();
        }
    }

    //Loop
    public void runLoop() {
        if (capModeButton.getState()) {
            robot.depositorLift.setCap(true);
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );

            if (driver1.liftUpCap) {
                robot.depositorLift.setHold(true);
                robot.depositorLift.manualLiftUp();
            }

//            if (driver1.raiseLift > 0) {
//                robot.depositorLift.setHold(true);
//                robot.depositorLift.manualLiftUp();
//                telemetry.addLine("Lifting up");
//            } else if (driver1.lowerLiftSlow) {
//                robot.depositorLift.setHold(false);
//                robot.depositorLift.slowLiftDown();
//                telemetry.addLine("Lifting down slow");
//            } else if (driver1.lowerLift > 0) {
//                robot.depositorLift.setHold(false);
//                robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
//                telemetry.addLine("Lifting down");
//            } else if (robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.STOP) {
//                robot.depositorLift.manualLiftHold();
//                telemetry.addLine("Holding");
//            }
        } else {
            robot.depositorLift.setCap(false);
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.9
                    )
            );

            if (driver1.collectOn) {
                retractFirstTime = false;
                if (!deployFirstTime) {
                    robot.collector.setGoal(Collector.Goal.DEPLOY);
                    deployFirstTime = true;
                }
            } else {
                deployFirstTime = false;
                if (!retractFirstTime) {
                    robot.collector.setGoal(Collector.Goal.RETRACT);
                    retractFirstTime = true;
                }
            }

            if (driver1.gateOverride) {
                telemetry.addLine("overriding");
                robot.collector.setGateOverride(true);
            } else {
                robot.collector.setGateOverride(false);
            }

            if (driver1.reverseCollect) {
                robot.collector.setSign(-1);
            } else {
                robot.collector.setSign(1);
            }

            if (depositButton.getState()) {
                if (extended) {
                    robot.depositorLift.open();
                    extended = false;
                } else {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                    extended = true;
                }
            }

            if (driver1.retract) {
                robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
                extended = false;
            }

            if (extendAdjustOutButton.getState()) {
                robot.depositorLift.adjustExtend(EXTEND_ADJUST_INTERVAL);
            } else if (extendAdjustInButton.getState()) {
                robot.depositorLift.adjustExtend(-EXTEND_ADJUST_INTERVAL);
            }

            if ((turretAdjustLeftButton.getState() && color == AllianceColor.BLUE)
                    || (turretAdjustRightButton.getState() && color == AllianceColor.RED)) {
                robot.turret.adjustTurret(TURRET_ADJUST_INTERVAL);
            } else if ((turretAdjustLeftButton.getState() && color == AllianceColor.RED)
                    || (turretAdjustRightButton.getState() && color == AllianceColor.BLUE)) {
                robot.turret.adjustTurret(-TURRET_ADJUST_INTERVAL);
            }

            if (driver2.depositHigh) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
            } else if (driver2.depositMid) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.MIDDLE);
            } else if (driver2.depositLow) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
            }
        }

        if (carouselButton.getState()) {
            robot.carouselSpin.on(color);
        } else {
            robot.carouselSpin.off();
        }

        telemetry.addData("Running", "Now");
        telemetry.addData("Deposit level", robot.depositorLift.getHeight());
        telemetry.addData("Lift encoder", robot.depositorLift.getLiftPosition());
        telemetry.addData("Extend encoder", robot.depositorLift.getExtendPosition());
        telemetry.addData("Lift limit", robot.depositorLift.isTouchPressed());
        telemetry.addData("Turret encoder", robot.turret.encoderPosition());
        telemetry.addData("Extend current draw", robot.depositorLift.getExtendCurrentDraw());
        telemetry.update();
    }

    private void mapControls(Driver1 driver1, Driver2 driver2) {
        ////////////
        //DRIVER 1//
        ////////////

        //Scale the input of the right stick in the x direction
        driver1.collectOn = collectOnButton.update(gamepad1.right_bumper);
        driver1.reverseCollect = gamepad1.left_bumper;
        driver1.gateOverride = gamepad1.a;

        depositButton.update(gamepad1.x);
        driver1.retract = gamepad1.b;

        capModeButton.update(gamepad1.y);

        driver1.liftUpCap = gamepad1.right_trigger > 0;
        driver1.liftDownCap = gamepad1.left_trigger > 0;

        driver1.extendOutCap = gamepad1.right_bumper;
        driver1.extendRetractCap = gamepad1.left_bumper;

        extendAdjustOutButton.update(gamepad1.dpad_up);
        extendAdjustInButton.update(gamepad1.dpad_down);

        turretAdjustLeftButton.update(gamepad1.dpad_left);
        turretAdjustRightButton.update(gamepad1.dpad_right);

        ////////////
        //DRIVER 2//
        ////////////

        carouselButton.update(gamepad2.a);
        driver2.depositHigh = gamepad2.dpad_up;
        driver2.depositMid = gamepad2.dpad_left;
        driver2.depositLow = gamepad2.dpad_down;
    }
}
