package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class CachingServo extends ServoImplEx {
    private double lastPosition = 0;

    public CachingServo(ServoImplEx servo) {
        super((ServoControllerEx) servo.getController(), servo.getPortNumber(), ServoConfigurationType.getStandardServoType());
    }

    @Override
    public synchronized void setPosition(double position) {
        if (position != lastPosition) {
            super.setPosition(position);
            lastPosition = position;
        }
    }
}
