package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

import static java.lang.Thread.sleep;
/*
           Collector
        ----|||-----
    /        0           \
|                            |
|                            |
|    444                     |
 */
public class Turret implements Component {
    private DcMotorEx turret;
    private DigitalChannel limit;

    private static final int RESET_TICKS = 0;
    private static final int DEPOSIT_TICKS_RED = 444;
    private static final double TURRET_POWER = 0.4;
    private static final double TURRET_POWER_SLOW = 0.15;

    private int turretDepositTicks = DEPOSIT_TICKS_RED;
    private Telemetry telemetry;

    public Turret (HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        turret = new CachingMotor(map.get(DcMotorEx.class, "turret"));

        limit = map.digitalChannel.get("turretLimit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setTargetPositionTolerance(3);
    }

    @Override
    public void reset() {
        stopTurret();
    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void resetTurret() throws InterruptedException {
        //TODO: fix
        Log.d("Turret", "Resetting");
        while(!limit.getState()) {
            turret.setPower(0.5);
        }
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTurretHoldPower() {
        turret.setPower(0.1);
    }

    public void spinTurret(Direction direction) {
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER : TURRET_POWER);
    }

    public void spinTurretDeposit() {
        turret.setTargetPosition(turretDepositTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_POWER);
    }

    public void adjustTurret(int ticks) {
        turretDepositTicks += ticks;
        turret.setTargetPosition(turretDepositTicks);
    }

    public void spinTurretReset() {
        turretDepositTicks = DEPOSIT_TICKS_RED;
        turret.setTargetPosition(RESET_TICKS);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(-TURRET_POWER);
    }

    public void spinTurretSlow(Direction direction) {
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER_SLOW : TURRET_POWER_SLOW);
    }

    public void stopTurret() {
        turret.setPower(0);
    }

    public int encoderPosition() {
        return turret.getCurrentPosition();
    }

    public int getTargetPosition() {
        return turret.getTargetPosition();
    }

    public boolean limitState() {
        return limit.getState();
    }

    public boolean isTurretBusy() {
        return turret.isBusy();
    }
}
