package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        robot.depositorLift.setHeight(DepositorLift.DepositorHeight.LOW);
        robot.depositorLift.setCap(false);
        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Lift ticks", robot.depositorLift.getLiftPosition());
            telemetry.update();
        }
    }

}
