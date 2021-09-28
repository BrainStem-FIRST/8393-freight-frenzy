package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

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

    private double driveInterpolationFactor = 4;

    private ToggleButton tb = new ToggleButton();

    private StickyButton sb = new StickyButton();

    private Driver1 driver1 = new Driver1();
    private Driver2 driver2 = new Driver2();

    private BrainSTEMRobot robot;

    protected AllianceColor color = null;

    private static final double THRESHOLD = 0;

    private class Driver1 {
        private boolean toggleReverseDrive;
        private boolean toggle;
        private boolean collect;
        private double drive, strafe, turn;
    }

    private class Driver2 {

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

        if ((Math.abs(leftStickX) > THRESHOLD) || Math.abs(rightStickX) > THRESHOLD) {
            //Set r equal to the magnitude of the input of the y stick in both x and y directions
            r = Math.pow(Math.sqrt(Math.pow(leftStickX, driveInterpolationFactor) + Math.pow(leftStickY, driveInterpolationFactor)), driveInterpolationFactor);

            //Calculate the angle between the y value of the left stick and the x value of the left stick
            theta = Math.atan2(leftStickY, leftStickX);
            //Calculate how much the robot needs to turn by, by determining if the right stick is above
            //the dead zone THRESHOLD
            turning = Math.abs(rightStickX) > THRESHOLD ? rightStickX : 0;

            //Set the speeds of the motors using the magnitude of the speed and the angle
//            robot.drive.setPower(r, theta, turning);
        } else
            robot.drive.setMotorPowers(0, 0);


        if(driver1.collect) {

        } else {

        }

        if(tb.getState()) {

        } else {

        }


        telemetry.addData("Running", "Now");
        telemetry.update();
    }

    private void mapControls(Driver1 driver1, Driver2 driver2) {
        ////////////
        //DRIVER 1//
        ////////////

        /*
        Right trigger: both collect on
        Left trigger: front collect reverse
        Left bumper: transfer + shooter on/off toggle
        Right bumper: shooter trigger
        y: tilt up
        a: tilt down
        Left stick button: turret left
        Right stick button: turret right
         */

        /*
        Left trigger: turret left
        Right trigger: turret right
        y: tilt up
        a: tilt down
         */

        //driver1.toggleReverseDrive = reverseDriveToggle.update(gamepad1.dpad_up);

        driver1.drive = gamepad1.left_stick_y;
        driver1.strafe = gamepad1.left_stick_x;
        driver1.turn = gamepad1.right_stick_x;

        //Reverse the inputs of the joysticks so that front and back switch
        if (!driver1.toggleReverseDrive) {
            leftStickX = leftXTuning * driver1.strafe;
            leftStickY = -leftYTuning * driver1.drive;
        } else {
            leftStickX = -leftXTuning * driver1.strafe;
            leftStickY = leftYTuning * driver1.drive;
        }

        //Scale the input of the right stick in the x direction
        rightStickX = rightXTuning * driver1.turn;

        driver1.collect = gamepad1.a;
        sb.update(gamepad1.left_stick_button);
        driver1.toggle = tb.update(gamepad1.b);

        ////////////
        //DRIVER 2//
        ////////////


    }
}
