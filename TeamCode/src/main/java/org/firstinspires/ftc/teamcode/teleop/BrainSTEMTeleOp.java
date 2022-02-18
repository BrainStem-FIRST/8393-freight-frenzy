package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.util.Direction;

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
    private ToggleButton straightModeButton = new ToggleButton();
    private ToggleButton depositorGateCapButton = new ToggleButton();
    private ToggleButton turboButton = new ToggleButton();
    private ToggleButton fullDepositButton = new ToggleButton();
    private ToggleButton carouselButton = new ToggleButton();

    private StickyButton depositButton = new StickyButton();
    private StickyButton extendAdjustOutButton = new StickyButton();
    private StickyButton extendAdjustInButton = new StickyButton();
    private StickyButton turretAdjustLeftButton = new StickyButton();
    private StickyButton turretAdjustRightButton = new StickyButton();
    private StickyButton manualDepositorRetractButton = new StickyButton();
    private StickyButton manualTurretResetLeftButton = new StickyButton();
    private StickyButton manualTurretResetRightButton = new StickyButton();

    private Driver1 driver1 = new Driver1();
    private Driver2 driver2 = new Driver2();

    private BrainSTEMRobot robot;

    protected AllianceColor color = null;

    private boolean extended = false;
    private boolean straightAdjustFirstTime = true;
    private boolean deployFirstTime = false, retractFirstTime = false;
    private boolean isDeployCap = false;

    private class Driver1 {
        private boolean collectOn;
        private boolean gateOverride;
        private boolean reverseCollect;
        private boolean retract;
        private boolean liftUpCap, liftDownCap;
        private boolean extendOutCap, extendRetractCap;
        private boolean turretLeftCap, turretRightCap;
    }

    private class Driver2 {
        private boolean depositHigh, depositMid, depositLow;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this);
        robot.reset();
        BrainSTEMRobot.mode = BrainSTEMRobot.Mode.ANGLED;
        robot.turret.setColor(color);
        robot.turret.setAuto(false);
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
            BrainSTEMRobot.mode = BrainSTEMRobot.Mode.CAP;
            robot.depositorLift.setHold(true);
            telemetry.addLine("in cap mode");
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.5,
                            -gamepad1.left_stick_x * 0.5,
                            -gamepad1.right_stick_x * 0.5
                    )
            );

            if (depositButton.getState()) {
                Log.d("BrainSTEM", "deposit button true");
                robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                isDeployCap = true;
            }

            if (isDeployCap) {
                if (driver1.liftUpCap) {
                    robot.depositorLift.manualLiftUp();
                } else if (driver1.liftDownCap) {
                    robot.depositorLift.manualLiftDown();
                } else {
                    robot.depositorLift.manualLiftHold();
                }

                if (driver1.extendOutCap) {
                    robot.depositorLift.manualExtendOutCap();
                } else if (driver1.extendRetractCap) {
                    robot.depositorLift.manualExtendBackCap();
                } else {
                    robot.depositorLift.stopExtend();
                }
                if (driver1.turretLeftCap) {
                    robot.turret.spinTurretCap(Direction.LEFT);
                } else if (driver1.turretRightCap) {
                    robot.turret.spinTurretCap(Direction.RIGHT);
                } else {
                    robot.turret.stopTurret();
                }
            }

            if (depositorGateCapButton.getState()) {
                robot.depositorLift.gateCap();
            } else {
                robot.depositorLift.close();
            }

        } else {
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

            if (manualDepositorRetractButton.getState()) {
                robot.depositorLift.manualExtendBack();
            } else if (manualDepositorRetractButton.getOppositeState()) {
                robot.depositorLift.resetExtendEncoder();
            }

            //these directions are correct if driver 2 stands near the ducks (looking at robot from back)
            if (manualTurretResetLeftButton.getState()) {
                robot.turret.spinTurretZeroAdjust(Direction.LEFT);
            } else if (manualTurretResetRightButton.getState()) {
                robot.turret.spinTurretZeroAdjust(Direction.RIGHT);
            } else if (manualTurretResetLeftButton.getOppositeState() || manualTurretResetRightButton.getOppositeState()) {
                robot.turret.resetTurretEncoder();
            }

            if (straightModeButton.getState()) {
                BrainSTEMRobot.mode = BrainSTEMRobot.Mode.STRAIGHT;
            } else {
                BrainSTEMRobot.mode = BrainSTEMRobot.Mode.ANGLED;
            }

            if (depositButton.getState()) {
                if (extended) {
                    if(fullDepositButton.getState()) {
                        robot.depositorLift.openFull();
                    } else {
                        robot.depositorLift.openPartial();
                    }
                    extended = false;
                } else {
                    robot.depositorLift.setGoal(DepositorLift.DepositorGoal.DEPLOY);
                    extended = true;
                }
            }

            if (driver1.retract) {
                straightAdjustFirstTime = true;
                robot.depositorLift.setGoal(DepositorLift.DepositorGoal.RETRACT);
                extended = false;
            }

            switch(BrainSTEMRobot.mode) {
                case ANGLED:
                    if (extendAdjustOutButton.getState()) {
                        robot.depositorLift.adjustExtend(EXTEND_ADJUST_INTERVAL);
                    } else if (extendAdjustInButton.getState()) {
                        robot.depositorLift.adjustExtend(-EXTEND_ADJUST_INTERVAL);
                    }

                    if (turretAdjustRightButton.getState()) {
                        robot.turret.adjustTurret(TURRET_ADJUST_INTERVAL);
                    } else if (turretAdjustLeftButton.getState()) {
                        robot.turret.adjustTurret(-TURRET_ADJUST_INTERVAL);
                    }
                    break;
                case STRAIGHT:
                    if (driver1.extendOutCap) {
                        straightAdjustFirstTime = false;
                        robot.depositorLift.manualExtendOutCap();
                    } else if (driver1.extendRetractCap) {
                        straightAdjustFirstTime = false;
                        robot.depositorLift.manualExtendBackCap();
                    } else if (!straightAdjustFirstTime) {
                        robot.depositorLift.stopExtend();
                    }
                    if (driver1.turretLeftCap) {
                        straightAdjustFirstTime = false;
                        robot.turret.spinTurretCap(Direction.LEFT);
                    } else if (driver1.turretRightCap) {
                        straightAdjustFirstTime = false;
                        robot.turret.spinTurretCap(Direction.RIGHT);
                    } else if (!straightAdjustFirstTime) {
                        robot.turret.stopTurret();
                    }
                    break;
            }

            if (driver2.depositHigh) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTHREE);
            } else if (driver2.depositMid) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELTWO);
            } else if (driver2.depositLow) {
                robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LEVELONE);
            }
        }

        if (carouselButton.getState()) {
            robot.carouselSpin.on(color);
        } else {
            robot.carouselSpin.off();
        }

        telemetry.addData("Running", "Now");
        telemetry.addData("Deposit level", robot.depositorLift.getHeight());
        telemetry.addData("Depositor mode", BrainSTEMRobot.mode);
        telemetry.addData("Extend power", robot.depositorLift.getExtendPower());
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

        driver1.extendOutCap = gamepad1.dpad_up;
        driver1.extendRetractCap = gamepad1.dpad_down;

        driver1.turretLeftCap = gamepad1.dpad_left;
        driver1.turretRightCap = gamepad1.dpad_right;
        turboButton.update(gamepad1.dpad_up);

        depositorGateCapButton.update(gamepad1.b);

        extendAdjustOutButton.update(gamepad1.dpad_up);
        extendAdjustInButton.update(gamepad1.dpad_down);

        turretAdjustLeftButton.update(gamepad1.dpad_left);
        turretAdjustRightButton.update(gamepad1.dpad_right);

        ////////////
        //DRIVER 2//
        ////////////

        carouselButton.update(gamepad2.a);
        straightModeButton.update(gamepad2.y);
        driver2.depositHigh = gamepad2.dpad_up;
        driver2.depositMid = gamepad2.dpad_left;
        driver2.depositLow = gamepad2.dpad_down;
        fullDepositButton.update(gamepad2.x);
        manualDepositorRetractButton.update(gamepad2.right_trigger > 0);
        manualTurretResetLeftButton.update(gamepad2.left_bumper);
        manualTurretResetRightButton.update(gamepad2.right_bumper);
    }
}
