package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;
import org.firstinspires.ftc.teamcode.util.Direction;

public class PixyCam implements Component {

    public double tse_x;
    public double tse_y;

    /*
    I2C values
    private static final double LEFT_X_THRESHOLD_RED = 15;
    private static final double MIDDLE_X_THRESHOLD_RED = 69;
    private static final double RIGHT_X_THRESHOLD_RED = 132;

    private static final double LEFT_X_THRESHOLD_BLUE = 0;
    private static final double MIDDLE_X_THRESHOLD_BLUE = 0;
    private static final double RIGHT_X_THRESHOLD_BLUE = 0;
    */

    private static final double LEFT_X_THRESHOLD_RED = 2.29;
    private static final double CENTER_X_THRESHOLD_RED = 2.41;
    private static final double RIGHT_X_THRESHOLD_RED = 2.65;

    private static final double LEFT_X_THRESHOLD_BLUE = 2.65;
    private static final double CENTER_X_THRESHOLD_BLUE = 2.985;
    private static final double RIGHT_X_THRESHOLD_BLUE = 3.0;

    private static final double MINIMUM_X = 0.1;

    private double leftXThreshold;
    private double centerXThreshold;
    private double rightXThreshold;

    private I2cDeviceSynch pixyCam;
    private AnalogInput pixyCamAnalog;
    private BarcodePattern pos;
    private AllianceColor color;

    public PixyCam(HardwareMap map, AllianceColor color) {
        this.color = color;
//        pixyCam = map.i2cDeviceSynch.get("Pixy Cam");
        pixyCamAnalog = map.analogInput.get("Pixy Cam");
        switch(color) {
            case RED:
                leftXThreshold = LEFT_X_THRESHOLD_RED;
                centerXThreshold = CENTER_X_THRESHOLD_RED;
                rightXThreshold = RIGHT_X_THRESHOLD_RED;
                break;
            case BLUE:
                leftXThreshold = LEFT_X_THRESHOLD_BLUE;
                centerXThreshold = CENTER_X_THRESHOLD_BLUE;
                rightXThreshold = RIGHT_X_THRESHOLD_BLUE;
                break;
        }
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
    /*
    I2C functions
    public BarcodePattern tsePos() {
        if (tse_x >= (centerXThreshold + rightXThreshold) / 2.0 && tse_x > minimumX) {
            pos = BarcodePattern.LEVELTHREE;
        } else if (tse_x >= (leftXThreshold + centerXThreshold) / 2.0  && tse_x > minimumX) {
            pos = BarcodePattern.LEVELTWO;
        } else {
            pos = BarcodePattern.LEVELONE;
        }
        return pos;


    public void teamShippingElementUpdate() {
        tse_x = 0xff&pixyCam.read(0x54,5)[1];
        tse_y = 0xff&pixyCam.read(0x54,5)[2];
    }

    public boolean isPixyEngaged() {
        return pixyCam.isEngaged();
    }

    public HardwareDeviceHealth.HealthStatus getHealth() {
        return pixyCam.getHealthStatus();
    }

    public String getConnectionInfo() {
        return pixyCam.getConnectionInfo();
    }
    */

    public BarcodePattern tsePos(double x) {
        if (x >= (centerXThreshold + rightXThreshold) / 2.0 && x > MINIMUM_X) {
            pos = BarcodePattern.LEVELTHREE;
        } else if (x >= (leftXThreshold + centerXThreshold) / 2.0  && x > MINIMUM_X) {
            pos = BarcodePattern.LEVELTWO;
        } else {
            pos = BarcodePattern.LEVELONE;
        }
        return pos;
    }

    public void teamShippingElementUpdate() {
        double val = pixyCamAnalog.getVoltage();
        if (color == AllianceColor.RED || pixyCamAnalog.getVoltage() > 2.4) {
            tse_x = val;
        }
    }

    public void setThreshold(Direction direction, double threshold) {
        switch (direction) {
            case LEFT:
                leftXThreshold = threshold;
                break;
            case CENTER:
                centerXThreshold = threshold;
                break;
            case RIGHT:
                rightXThreshold = threshold;
                break;
        }
    }

    public double getThreshold(Direction direction) {
        switch (direction) {
            case LEFT:
                return leftXThreshold;
            case CENTER:
                return centerXThreshold;
            case RIGHT:
                return rightXThreshold;
        }
        return 0;
    }
}
