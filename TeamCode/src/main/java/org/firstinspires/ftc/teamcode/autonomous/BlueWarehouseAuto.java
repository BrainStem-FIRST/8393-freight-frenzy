package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueWarehouseAuto extends BrainSTEMAutonomous {
    public BlueWarehouseAuto() {
        super();
        color = AllianceColor.BLUE;
        startLocation = StartLocation.WAREHOUSE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}