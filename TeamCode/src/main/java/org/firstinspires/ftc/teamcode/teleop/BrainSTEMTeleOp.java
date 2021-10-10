package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;

import java.util.List;

@TeleOp
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

    private static final double THRESHOLD = 0;

    private boolean extended = false;

    private class Driver1 {
        private boolean toggleReverseDrive;
        private boolean collectOn;
        private boolean reverseCollect;
        private boolean raiseLift;
        private boolean lowerLift;
        private boolean retract;
        private double drive, turn;
    }

    private class Driver2 {
        private boolean spinCarousel;
        private double aimTurret;
        private boolean depositNear, depositFar;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this);

        robot.init();
        while (!opModeIsActive() && !isStopRequested()) {
            //Status to show if telemetry was initialized
            telemetry.addData("Status", "Initialized");
//            telemetry.addData("IMU Calibrated during Loop?", robot.drive.getCalibrated());
            telemetry.update();
        }

        robot.start();
        while(opModeIsActive()) {
            runLoop();
        }
    }

    //Loop
    public void runLoop() {
        mapControls(driver1, driver2);

        if ((Math.abs(leftStickY) > THRESHOLD) || Math.abs(rightStickX) > THRESHOLD) {
           double leftPower = Math.pow(leftStickY, driveInterpolationFactor) + rightStickX;
           double rightPower = Math.pow(leftStickY, driveInterpolationFactor) - rightStickX;

           double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
           if (max > 1) {
               leftPower /= max;
               rightPower /= max;
           }

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            robot.drive.setMotorPowers(leftPower, rightPower);
        } else robot.drive.setMotorPowers(0, 0);


//        if(driver1.collectOn) {
//            robot.collector.startCollection();
//        } else {
//            robot.collector.stopCollection();
//        }
//
//        if (driver1.reverseCollect) {
//            robot.collector.reverse();
//        }
//
//        if (driver2.depositNear) {
//            robot.depositorLift.setDepositLocation(DepositorLift.Location.NEAR);
//        } else if (driver2.depositFar) {
//            robot.depositorLift.setDepositLocation(DepositorLift.Location.FAR);
//        }
//
//        if(depositButton.getState()) {
//            if (extended) {
//                robot.depositorLift.deposit();
//                extended = false;
//            } else {
//                robot.depositorLift.extend();
//                extended = true;
//            }
//        }
//
//        if (driver1.retract) {
//            robot.depositorLift.retractDeposit();
//            robot.depositorLift.retractExtension();
//        }
//
//        if(driver1.raiseLift) {
//            robot.depositorLift.setGoal(DepositorLift.Goal.UP);
//        } else if (driver1.lowerLift) {
//            robot.depositorLift.setGoal(DepositorLift.Goal.DOWN);
//        }
//
//        robot.turret.autoSpinTurret(Math.toDegrees(driver2.aimTurret));

        telemetry.addData("Running", "Now");
        telemetry.update();
    }

    private void mapControls(Driver1 driver1, Driver2 driver2) {
        ////////////
        //DRIVER 1//
        ////////////

        //driver1.toggleReverseDrive = reverseDriveToggle.update(gamepad1.dpad_up);

        driver1.drive = gamepad1.left_stick_y;
        driver1.turn = gamepad1.right_stick_x;

        //Reverse the inputs of the joysticks so that front and back switch
        if (!driver1.toggleReverseDrive) {
            leftStickY = -leftYTuning * driver1.drive;
        } else {
            leftStickY = leftYTuning * driver1.drive;
        }

        //Scale the input of the right stick in the x direction
        rightStickX = rightXTuning * driver1.turn;

        driver1.collectOn = collectOnButton.update(gamepad1.right_trigger > THRESHOLD);
        driver1.reverseCollect = gamepad1.left_trigger > THRESHOLD;
        driver1.raiseLift = gamepad1.right_bumper;
        driver1.lowerLift = gamepad1.left_bumper;

        depositButton.update(gamepad1.x);
        driver1.retract = gamepad1.b;

        ////////////
        //DRIVER 2//
        ////////////

        driver2.depositNear = gamepad2.dpad_down;
        driver2.depositFar = gamepad2.dpad_up;
        driver2.spinCarousel = gamepad2.right_bumper;
        if (gamepad2.right_stick_x > 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / gamepad2.right_stick_x);
        } else if (gamepad2.right_stick_x < 0) {
            driver2.aimTurret = Math.atan(gamepad2.right_stick_y / gamepad2.right_stick_x) + Math.PI;
        } else {
            driver2.aimTurret = Math.PI / 2;
        }
    }
}
