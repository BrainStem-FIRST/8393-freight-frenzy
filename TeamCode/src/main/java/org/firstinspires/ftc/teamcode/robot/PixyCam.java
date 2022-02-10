package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;

public class PixyCam implements Component {

    public double tse_x;
    public double tse_y;

    private static final double LEFT_X_THRESHOLD_RED = 15;
    private static final double MIDDLE_X_THRESHOLD_RED = 69;
    private static final double RIGHT_X_THRESHOLD_RED = 132;

    private static final double LEFT_X_THRESHOLD_BLUE = 0;
    private static final double MIDDLE_X_THRESHOLD_BLUE = 0;
    private static final double RIGHT_X_THRESHOLD_BLUE = 0;

    private double leftXThreshold;
    private double middleXThreshold;
    private double rightXThreshold;

    private I2cDeviceSynch pixyCam;
    private BarcodePattern pos;

    public PixyCam(HardwareMap map) {
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

    }

    @Override
    public String test() {
        return null;
    }

    public BarcodePattern tsePos() {
        if (tse_x >= (middleXThreshold + rightXThreshold) / 2.0 && tse_x > 20) {
            pos = BarcodePattern.LEVELTHREE;
        } else if (tse_x >= (leftXThreshold + middleXThreshold) / 2.0  && tse_x > 20) {
            pos = BarcodePattern.LEVELTWO;
        } else {
            pos = BarcodePattern.LEVELONE;
        }
        return pos;
    }

    public void teamShippingElementUpdate() {
        tse_x = 0xff&pixyCam.read(0x51,5)[1];
        tse_y = 0xff&pixyCam.read(0x51,5)[2];
    }

    public boolean isPixyEngaged() {
        return pixyCam.isEngaged();
    }

    public void setColor(AllianceColor color) {
        switch(color) {
            case RED:
                leftXThreshold = LEFT_X_THRESHOLD_RED;
                middleXThreshold = MIDDLE_X_THRESHOLD_RED;
                rightXThreshold = RIGHT_X_THRESHOLD_RED;
                break;
            case BLUE:
                leftXThreshold = LEFT_X_THRESHOLD_BLUE;
                middleXThreshold = MIDDLE_X_THRESHOLD_BLUE;
                rightXThreshold = RIGHT_X_THRESHOLD_BLUE;
                break;
        }
    }
    //Actual Values
    //left: 15
    //center: 69
    //right: 132
}
