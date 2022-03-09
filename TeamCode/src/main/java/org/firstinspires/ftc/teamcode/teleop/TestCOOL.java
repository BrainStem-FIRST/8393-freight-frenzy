package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.COOLLocalizer;
@TeleOp
public class TestCOOL extends LinearOpMode {
    public COOLLocalizer cool;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap map = this.hardwareMap;
        cool = new COOLLocalizer(map);

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("x COOL", cool.getPoseEstimate().getX());
            telemetry.addData("y COOL", cool.getPoseEstimate().getY());
            telemetry.addData("pose heading COOL", cool.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
