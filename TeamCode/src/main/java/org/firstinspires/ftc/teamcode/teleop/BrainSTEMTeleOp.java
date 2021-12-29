package org.firstinspires.ftc.teamcode.teleop;

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
        private boolean reverseCollect;
        private float raiseLift;
        private float lowerLift;
        public boolean upCollector = false;
        private boolean retract;
        private double drive, turn;
    }

    private class Driver2 {
        private boolean spinCarousel;
        private double aimTurret;
        private float turretLeft;
        private float turretRight;
        private boolean teamShippingElement;
        private boolean depositHigh, depositLow;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this);

        robot.reset();
        robot.depositorLift.clampSE();
        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
//            telemetry.addData("IMU Calibrated during Loop?", robot.drive.getCalibrated());
            telemetry.update();
        }

        while(opModeIsActive()) {
            robot.update();
            mapControls(driver1, driver2);
            runLoop();
        }
    }

    //Loop
    public void runLoop() {
        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        double left  = drive + turn;
        double right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe values to the motor drives.
        robot.drive.setMotorPowers(left, right);

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
                robot.depositorLift.setGoal(DepositorLift.Goal.DEPLOY);
                extended = true;
            }
        }

        if (driver1.retract) {
            robot.depositorLift.setGoal(DepositorLift.Goal.RETRACT);
            extended = false;
        }

        if (gamepad1.y) {
            robot.depositorLift.close();
        }

        if (driver1.raiseLift > 0) {
            robot.depositorLift.liftUp();
        } else if (driver1.lowerLift > 0) {
            robot.depositorLift.liftDown();
        } else {
            robot.depositorLift.stopLift();
        }

        if (driver2.spinCarousel) {
            robot.carouselSpin.spinCarousel(color);
        } else {
            robot.carouselSpin.stopCarousel();
        }

        if (driver2.turretLeft > 0) {
            robot.turret.spinTurret(Direction.LEFT);
        } else if (driver2.turretRight > 0) {
            robot.turret.spinTurret(Direction.RIGHT);
        } else {
            robot.turret.stopTurret();
        }

        if (driver2.teamShippingElement) {
            robot.depositorLift.releaseSE();
        }

        if (driver2.depositHigh) {
            robot.depositorLift.setDepositLow(false);
        } else if (driver2.depositLow) {
            robot.depositorLift.setDepositLow(true);
        }

//        robot.turret.autoSpinTurret(Math.toDegrees(driver2.aimTurret));

        telemetry.addData("Running", "Now");
        telemetry.addData("Turret encoder", robot.turret.encoderPosition());
        telemetry.addData("Deposit first level?", robot.depositorLift.getDepositLow());
        telemetry.update();
    }

    private void mapControls(Driver1 driver1, Driver2 driver2) {
        ////////////
        //DRIVER 1//
        ////////////


        driver1.drive = gamepad1.left_stick_y;
        driver1.turn = gamepad1.right_stick_x;

        //Scale the input of the right stick in the x direction
        driver1.collectOn = collectOnButton.update(gamepad1.right_bumper);
        driver1.reverseCollect = gamepad1.left_bumper;
        driver1.raiseLift = gamepad1.right_trigger;
        driver1.lowerLift = gamepad1.left_trigger;
        depositButton.update(gamepad1.x);
        driver1.retract = gamepad1.b;


        ////////////
        //DRIVER 2//
        ////////////

        driver2.turretLeft = gamepad2.left_trigger;
        driver2.turretRight = gamepad2.right_trigger;
        driver2.spinCarousel = gamepad2.x;
        if (gamepad2.right_stick_x > 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / gamepad2.right_stick_x);
        } else if (gamepad2.right_stick_x < 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / gamepad2.right_stick_x) + Math.PI;
        } else {
            driver2.aimTurret = Math.PI / 2;
        }
        driver2.teamShippingElement = gamepad2.y;
        driver2.depositHigh = gamepad2.dpad_up;
        driver2.depositLow = gamepad2.dpad_down;
    }
}
