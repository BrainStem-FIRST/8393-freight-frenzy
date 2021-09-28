package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CachingMotor extends DcMotorImplEx {
    private double lastPower = 0;
    private boolean powerChanged = true;

    private int lastPosition = 0;

    private double lastAngularRate = 0;
    private boolean angularRateChanged = true;

    public CachingMotor(DcMotorEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    @Override
    public synchronized void setPower(double power) {
        if (powerChanged || power != lastPower) {
            super.setPower(power);
            lastPower = power;
            powerChanged = false;
            angularRateChanged = true;
        }
    }

    @Override
    public synchronized void setTargetPosition(int position) {
        if (position != lastPosition) {
            super.setTargetPosition(position);
            lastPosition = position;
        }
    }

    @Override
    public synchronized void setVelocity(double angularRate) {
        if (angularRateChanged || angularRate != lastAngularRate) {
            super.setVelocity(angularRate);
            lastAngularRate = angularRate;
            angularRateChanged = false;
            powerChanged = true;
        }
    }
}