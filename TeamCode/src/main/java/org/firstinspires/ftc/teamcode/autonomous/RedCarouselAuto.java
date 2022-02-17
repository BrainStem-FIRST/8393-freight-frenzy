package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class RedCarouselAuto extends BrainSTEMAutonomous {
    public RedCarouselAuto() {
        super();
        color = AllianceColor.RED;
        startLocation = StartLocation.CAROUSEL;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}