package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.CarouselSpin;
import org.firstinspires.ftc.teamcode.teleop.StickyButton;

@TeleOp
public class CarouselTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CarouselSpin spin = new CarouselSpin(hardwareMap);
        double power = 0.1;
        StickyButton button = new StickyButton();

        telemetry.addData("Status:", "ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            button.update(gamepad1.a);
            spin.setPower(power);
            if (button.getState()) {
                power += 0.1;
            }
            telemetry.addData("power", power);
            telemetry.update();
        }
    }

}
