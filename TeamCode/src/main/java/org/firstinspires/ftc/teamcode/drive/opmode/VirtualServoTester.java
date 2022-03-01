package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.StickyButton;
import org.firstinspires.ftc.teamcode.teleop.ToggleButton;

@TeleOp
public class VirtualServoTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int PWM = 1310;
        double pos;
        BrainSTEMRobot robot = new BrainSTEMRobot(this, AllianceColor.RED, false);
        ToggleButton button = new ToggleButton();
        ToggleButton button1 = new ToggleButton();
        StickyButton button2 = new StickyButton();
        StickyButton button3 = new StickyButton();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            button.update(gamepad1.a);
            button1.update(gamepad1.b);
            button2.update(gamepad1.dpad_up);
            button3.update(gamepad1.dpad_down);
            telemetry.addLine("Gamepad 1 A: turret lock on");
            telemetry.addLine("Gamepad 1 B: depositor flip on");
            telemetry.addLine("Gamepad 1 Dpad Up: PWM up by 5");
            telemetry.addLine("Gamepad 1 Dpad Down: PWM down by 5");

            if (button2.getState()) {
                PWM += 5;
            }
            if(button3.getState()) {
                PWM -= 5;
            }

            pos = (double)(PWM-100)/(2520-100);

            if (button.getState() && !button1.getState()) {
                telemetry.addLine("Turret lock");
//                robot.turret.setLockExternal(pos);
            } else if (button1.getState() && !button.getState()) {
                telemetry.addLine("Depositor flip");
                robot.depositorLift.setRotateExternal(pos);
            }
            telemetry.addData("Pos", pos);
            telemetry.addData("PWM", PWM);
            telemetry.update();
        }
    }
}
