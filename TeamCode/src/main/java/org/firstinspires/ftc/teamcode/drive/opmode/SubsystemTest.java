package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.teleop.StickyButton;
import org.firstinspires.ftc.teamcode.teleop.ToggleButton;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        StickyButton button = new StickyButton();
        StickyButton button1 = new StickyButton();
        double position = 0.5;
        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            button.update(gamepad1.y);
            button1.update(gamepad1.a);
            if (button.getState()) {
                position += 0.05;
            }
            if (button1.getState()) {
                position -= 0.05;
            }
            robot.depositorLift.test(position);
            telemetry.addLine("Extend: " + robot.depositorLift.getExtendPosition());
            telemetry.addLine("Lift: " + robot.depositorLift.getLiftPosition());
            telemetry.addLine("Position of gate: " + position);
            telemetry.update();
        }
    }
}
