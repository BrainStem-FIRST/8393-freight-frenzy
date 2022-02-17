package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class BlueCarouselAuto extends BrainSTEMAutonomous {
    public BlueCarouselAuto() {
        super();
        color = AllianceColor.BLUE;
        startLocation = StartLocation.CAROUSEL;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }
}