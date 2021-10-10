package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedWarehouseAuto extends BrainSTEMAutonomous {
    public RedWarehouseAuto() {
        super();
        color = AllianceColor.RED;
        startLocation = StartLocation.WAREHOUSE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}