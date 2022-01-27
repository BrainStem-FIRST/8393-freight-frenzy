package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.DepositorLift;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
public class LimitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, new DepositorLift(hardwareMap), telemetry);
        waitForStart();
        turret.resetTurret();
    }

}
