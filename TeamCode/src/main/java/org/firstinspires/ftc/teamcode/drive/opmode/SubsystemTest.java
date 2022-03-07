package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//            robot.collector.on();
//            robot.update();
            button.update(gamepad1.x);
            button1.update(gamepad1.b);
            if (button.getState()) {
                robot.collector.close();
                telemetry.addLine("Closed");
//                robot.turret.spinTurretDeposit();
            } else {
                robot.collector.open();
                telemetry.addLine("Open");
            }
            if (button1.getState()) {
//                robot.turret.spinTurretReset();
            }
            telemetry.addLine("Collector color brightness: " + robot.collector.getBrightness());
            telemetry.addLine("Is freight detected? " + robot.collector.isFreightCollectedColor());
            telemetry.addLine("Depositor position: " + robot.depositorLift.getRotatePosition());
            telemetry.addLine("Extend: " + robot.depositorLift.getExtendPosition());
            telemetry.addLine("Lift: " + robot.depositorLift.getLiftPosition());
            telemetry.addLine("Turret: " + robot.turret.encoderPosition());
            telemetry.addLine("Turret current draw: " + robot.turret.getCurrentDraw());
            telemetry.addLine("Is turret current draw threshold triggered? " + robot.turret.isCurrentDrawPastThreshold());
            telemetry.addLine("Lift touch pressed? " + robot.depositorLift.isTouchPressed());
            telemetry.update();
        }
    }
}
