package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.StickyButton;
import org.firstinspires.ftc.teamcode.teleop.ToggleButton;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this, AllianceColor.RED, false);
        ToggleButton button = new ToggleButton();
        StickyButton button1 = new StickyButton();
        double power = 1;
        boolean first = true;
        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            robot.depositorLift.close();
//            robot.update();
            button.update(gamepad1.x);
            button1.update(gamepad1.b);
            if (button.getState()) {
//                robot.turret.spinTurretDeposit();
            } else {

            }
            if (button1.getState()) {
//                robot.turret.spinTurretReset();
            }
            telemetry.addLine("Extend: " + robot.depositorLift.getExtendPosition());
            telemetry.addLine("Lift: " + robot.depositorLift.getLiftPosition());
            telemetry.addLine("Turret: " + robot.turret.encoderPosition());
            telemetry.addLine("Lift touch pressed? " + robot.depositorLift.isTouchPressed());
            telemetry.update();
        }
    }
}
