package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;

@TeleOp
public class RedTeleOp extends BrainSTEMTeleOp {
    public RedTeleOp() {
        super();
        color = AllianceColor.RED;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}
