package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Collector;

@TeleOp
public class CollectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Collector collector = new Collector(this.hardwareMap, false);
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData("Status", "Ready");
        dashboardTelemetry.update();
        boolean firstTime = true;
        waitForStart();

        collector.setGoal(Collector.Goal.DEPLOY);

        while (opModeIsActive()) {
            collector.update();

            if(collector.isFreightCollectedColor() && firstTime == true)
            {
                collector.setGoal(Collector.Goal.RETRACT);
                firstTime = false;

            }
            dashboardTelemetry.addData("Current draw", collector.getCurrentDrawAverage());
            dashboardTelemetry.addData("Collected (current draw)?", collector.isFreightCollectedCurrentDraw());
            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("Color brightness", collector.getBrightness());
            dashboardTelemetry.addData("Collected (color)?", collector.isFreightCollectedColor());
            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("Collected (overall)?", collector.isFreightCollected());
            dashboardTelemetry.update();
        }
    }
}
