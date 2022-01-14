package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CarouselSpin;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.teleop.StickyButton;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        robot.depositorLift.setAutoSE(false);
        robot.depositorLift.setCap(false);
        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            robot.depositorLift.flipAutoSE();
            telemetry.addData("Lift ticks", robot.depositorLift.getLiftTicks());
            telemetry.update();
        }
    }

}
