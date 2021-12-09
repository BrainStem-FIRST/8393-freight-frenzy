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
        double power = 0.1;
        CarouselSpin spin = new CarouselSpin(hardwareMap);
        StickyButton sb1 = new StickyButton();
        StickyButton sb2 = new StickyButton();

        waitForStart();

        while (opModeIsActive()) {
            spin.spinCarousel(AllianceColor.BLUE);

            sb1.update(gamepad1.dpad_up);
            sb2.update(gamepad1.dpad_down);

            if (sb1.getState()) {
                if (power < 0.5) {
                    power += 0.01;
                }
            }

            if (sb2.getState()) {
                if (power > 0) {
                    power -= 0.01;
                }
            }

            telemetry.addLine("Power: " + power);
            telemetry.update();
        }
    }

}
