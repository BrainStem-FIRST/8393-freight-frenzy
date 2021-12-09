package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;

@TeleOp
public class BlueTeleOp extends BrainSTEMTeleOp {
    public BlueTeleOp() {
        super();
        color = AllianceColor.BLUE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}
