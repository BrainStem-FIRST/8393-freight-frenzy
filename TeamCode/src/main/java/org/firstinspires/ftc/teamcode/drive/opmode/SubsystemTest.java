package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
        Collector collector = new Collector(this.hardwareMap);
        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Is freight detected", collector.isFreightCollectedColor());
            telemetry.addData("Brightness value", collector.getBrightness());
            telemetry.update();
        }
    }

}
