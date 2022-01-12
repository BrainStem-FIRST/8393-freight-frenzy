package org.firstinspires.ftc.teamcode.teleop;

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
    private ToggleButton capButton1 = new ToggleButton();
    private ToggleButton capButton2 = new ToggleButton();

    private StickyButton depositButton = new StickyButton();

    private Driver1 driver1 = new Driver1();
    private Driver2 driver2 = new Driver2();

    private BrainSTEMRobot robot;

    protected AllianceColor color = null;

    private boolean extended = false;
    private boolean deployFirstTime = false, retractFirstTime = false;

    private class Driver1 {
        private boolean toggleReverseDrive;
        private boolean collectOn;
        private boolean gateOverride;
        private boolean reverseCollect;
        private float raiseLift;
        private boolean lowerLiftSlow;
        private float lowerLift;
        public boolean upCollector = false;
        private boolean retract;
        private double drive, turn, strafe;
        private boolean teamShippingElement;
    }

    private class Driver2 {
        private float spinCarousel;
        private double aimTurret;
        private float turretLeft;
        private float turretRight;
        private boolean depositHigh, depositMid, depositLow;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this);
//        robot.turret.setTurretHold();
        robot.reset();
        robot.depositorLift.clampSE();
        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
//            telemetry.addData("IMU Calibrated during Loop?", robot.drive.getCalibrated());
            telemetry.update();
        }
        robot.depositorLift.flipIn();
//        robot.turret.resetTurret();

        while(opModeIsActive()) {
            robot.update();
            mapControls(driver1, driver2);
            runLoop();
        }
    }

    //Loop
    public void runLoop() {
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

//        robot.turret.setTurretHoldPower();

        if(driver1.collectOn) {
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

        if(depositButton.getState()) {
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
//            robot.turret.autoSpinTurret(Math.PI);
        } else {
//            robot.turret.autoSpinTurret(driver2.aimTurret);
        }

        if (driver1.raiseLift > 0) {
            robot.depositorLift.setHold(true);
            robot.depositorLift.manualLiftUp();
            telemetry.addLine("Lifting up");
        } else if (driver1.lowerLiftSlow) {
            robot.depositorLift.setHold(false);
            robot.depositorLift.slowLiftDown();
            telemetry.addLine("Lifting down slow");
        } else if (driver1.lowerLift > 0) {
            robot.depositorLift.setHold(false);
            robot.depositorLift.setGoal(DepositorLift.LiftGoal.LIFTDOWN);
            telemetry.addLine("Lifting down");
        } else if (robot.depositorLift.getLiftGoal() == DepositorLift.LiftGoal.STOP) {
            robot.depositorLift.manualLiftHold();
            telemetry.addLine("Holding");
        }
//        } else if (!robot.depositorLift.limitState()) {
//            robot.depositorLift.manualLiftHold();
//            telemetry.addLine("Holding");
//        }

        if (driver2.spinCarousel > 0) {
            robot.carouselSpin.spinCarousel(color);
        } else {
            robot.carouselSpin.stopCarousel();
        }

        if(gamepad2.a)
        {
            robot.turret.backToZeroPosition();
        }

        if (driver2.turretLeft > 0) {
            robot.turret.spinTurretSlow(Direction.LEFT);
        } else if (driver2.turretRight > 0) {
            robot.turret.spinTurretSlow(Direction.RIGHT);
        } else {
            robot.turret.stopTurret();
        }

        if (driver1.teamShippingElement) {
            robot.depositorLift.releaseSE();
        }

        if (driver2.depositHigh) {
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.HIGH);
        } else if (driver2.depositMid) {
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.MIDDLE);
        } else if (driver2.depositLow) {
            robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        }

        if (capButton1.getState()) {
            robot.depositorLift.setCap(true);
        } else {
            robot.depositorLift.setCap(false);
        }

        telemetry.addData("Running", "Now");
//        telemetry.addData("Turret encoder", robot.turret.encoderPosition());
        telemetry.addData("Deposit level", robot.depositorLift.getHeight());
        telemetry.addData("Lift encoder", robot.depositorLift.getLiftTicks());
        telemetry.addData("Lift limit", robot.depositorLift.isTouchPressed());
        telemetry.addData("Turret limit", robot.turret.limitState());
        telemetry.addData("Lift power", robot.depositorLift.getLiftPower());
        telemetry.addData("Turret Encoder Value", robot.turret.encoderPosition());
        telemetry.update();
    }

    private void mapControls(Driver1 driver1, Driver2 driver2) {
        ////////////
        //DRIVER 1//
        ////////////

        //Scale the input of the right stick in the x direction
        driver1.collectOn = collectOnButton.update(gamepad1.right_bumper);
        driver1.reverseCollect = gamepad1.left_bumper;
        driver1.raiseLift = gamepad1.right_trigger;
        driver1.lowerLift = gamepad1.left_trigger;
        driver1.lowerLiftSlow = gamepad1.dpad_down;
        depositButton.update(gamepad1.x);
        driver1.retract = gamepad1.b;
        driver1.teamShippingElement = gamepad1.y;
        capButton1.update(gamepad1.a);
        driver1.gateOverride = gamepad1.dpad_up;


        ////////////
        //DRIVER 2//
        ////////////

        driver2.turretLeft = gamepad2.left_trigger;
        driver2.turretRight = gamepad2.right_trigger;
        driver2.spinCarousel = gamepad2.right_trigger;
        if (gamepad2.right_stick_x > 0 && gamepad2.right_stick_y > 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / gamepad2.right_stick_x) + Math.PI / 2;
        } else if (gamepad2.right_stick_x < 0 && gamepad2.right_stick_y > 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / -gamepad2.right_stick_x) + Math.PI;
        } else if (gamepad2.right_stick_x < 0 && gamepad2.right_stick_y < 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / -gamepad2.right_stick_x) + 3 * Math.PI / 2;
        } else if (gamepad2.right_stick_x > 0 && gamepad2.right_stick_y < 0) {
            driver2.aimTurret = Math.atan(-gamepad2.right_stick_y / gamepad2.right_stick_x);
        } else {
            driver2.aimTurret = Math.PI;
        }
        driver2.depositHigh = gamepad2.dpad_up;
        driver2.depositMid = gamepad2.dpad_left;
        driver2.depositLow = gamepad2.dpad_down;
        capButton2.update(gamepad2.a);
    }
}
