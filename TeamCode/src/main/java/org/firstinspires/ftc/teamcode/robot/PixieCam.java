package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;

public class PixieCam implements Component {

    public double tse_x;
    public double tse_y;

    public final double LEFT_X = 135;
    public final double MIDDLE_X = 197;
    public final double RIGHT_X = 240;

    private I2cDeviceSynch pixyCam;
    private BarcodePattern pos;

    public PixieCam(HardwareMap map) {
        pixyCam = map.i2cDeviceSynch.get("Pixy Cam");
    }

    @Override
    public void reset() {
    }

    public void start() {
        pixyCam.engage();
    }

    public void stop() {
        pixyCam.disengage();
    }

    @Override
    public void update() {
        tse_x = 0xff&pixyCam.read(0x51,5)[1];
        tse_y = 0xff&pixyCam.read(0x51,5)[2];
    }

    @Override
    public String test() {
        return null;
    }

    public BarcodePattern tsePos() {
        if (tse_x <= (LEFT_X + MIDDLE_X) / 2.0) {
            pos = BarcodePattern.LEVELONE;
        } else if (tse_x <= (MIDDLE_X + RIGHT_X) / 2.0) {
            pos = BarcodePattern.LEVELTWO;
        } else {
            pos = BarcodePattern.LEVELTHREE;
        }
        return pos;
    }
}