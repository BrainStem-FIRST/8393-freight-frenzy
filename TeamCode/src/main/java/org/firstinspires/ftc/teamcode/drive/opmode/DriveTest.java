package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
@Disabled
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("FL ticks", drive.getWheelPositions().get(0));
            telemetry.addData("BL ticks", drive.getWheelPositions().get(1));
            telemetry.addData("BR ticks", drive.getWheelPositions().get(2));
            telemetry.addData("FR ticks", drive.getWheelPositions().get(3));
            telemetry.update();
        }
    }

}
