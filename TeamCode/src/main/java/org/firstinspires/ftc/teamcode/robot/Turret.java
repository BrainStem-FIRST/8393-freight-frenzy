package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.Direction;

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
    private ServoImplEx lock;

    private static final int RESET_TICKS_BLUE = -10;
    private static final int RESET_TICKS_RED = 10;

    private static final int DEPOSIT_TICKS_RED = 710;
    private static final int DEPOSIT_TICKS_BLUE = -640;
    private static final int DEPOSIT_TICKS_BLUE_AUTO = -700;

    private static final int SHARED_TICKS_BLUE = 710;
    private static final int SHARED_TICKS_RED = -SHARED_TICKS_BLUE;

    private static final int CURRENT_THRESHOLD = 6000;

    private static final double TURRET_POWER_ADJUST = 1;
    private static final double TURRET_POWER = 0.8;
    private static final double TURRET_POWER_SLOW = 0.4;
    private static final double TURRET_POWER_MANUALRESET = 0.3;
    private static final PIDFCoefficients TURRET_PID = new PIDFCoefficients(20,0,1,0);

    private int turretDepositTicks = 0;
    private int resetTicks = 0;
    private AllianceColor color;
    private boolean isAuto = false;
    private boolean isTurretZero = true;
    private boolean blueAutoOverride = false;

    public Turret (HardwareMap map, AllianceColor color, boolean isAuto) {
        this.color = color;
        this.isAuto = isAuto;
        turret = new CachingMotor(map.get(DcMotorEx.class, "turret"));
        lock = new CachingServo(map.get(ServoImplEx.class, "lock"));

//        limit = map.digitalChannel.get("turretLimit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setTargetPositionTolerance(7);

        lock.setPwmRange(new PwmControl.PwmRange(1990, 2123));
    }

    @Override
    public void reset() {
        stopTurret();
//        if (isAuto) {
//            resetTicks = 0;
//        } else {
//            resetTicks = color == AllianceColor.BLUE ? RESET_TICKS_BLUE : RESET_TICKS_RED;
//        }
    }

    @Override
    public void update() {
    }

    @Override
    public String test() {
        return null;
    }

    public void spinTurretDeposit() {
        isTurretZero = false;
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (blueAutoOverride) {
            turretDepositTicks = DEPOSIT_TICKS_BLUE_AUTO;
        } else if (turretDepositTicks == 0 && BrainSTEMRobot.mode != BrainSTEMRobot.Mode.SHARED) {
            turretDepositTicks = color == AllianceColor.BLUE ? DEPOSIT_TICKS_BLUE : DEPOSIT_TICKS_RED;
        } else if (turretDepositTicks == 0) {
            //shared hub
            turretDepositTicks = color == AllianceColor.BLUE ? SHARED_TICKS_BLUE : SHARED_TICKS_RED;
        }
        turret.setTargetPosition(turretDepositTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, TURRET_PID);
        turret.setPower(TURRET_POWER);
    }

    public void spinTurretManual(double power) {
        turret.setPower(power);
    }

    public void adjustTurret(int ticks) {
        turretDepositTicks += ticks;
        turret.setPower(TURRET_POWER_ADJUST);
        turret.setTargetPosition(turretDepositTicks);
    }

    public void spinTurretReset() {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretDepositTicks = color == AllianceColor.BLUE ? DEPOSIT_TICKS_BLUE : DEPOSIT_TICKS_RED;
        turret.setPower(color == AllianceColor.BLUE ? TURRET_POWER : -TURRET_POWER);
    }

    public void spinTurretResetShared() {
        resetTicks = color == AllianceColor.BLUE ? RESET_TICKS_BLUE : RESET_TICKS_RED;
        turretDepositTicks = color == AllianceColor.BLUE ? SHARED_TICKS_BLUE : SHARED_TICKS_RED;
        turret.setTargetPosition(resetTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, TURRET_PID);
        turret.setPower(-TURRET_POWER);
    }

    public void spinTurretCap(Direction direction) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER_SLOW : TURRET_POWER_SLOW);
    }

    public void spinTurretZeroAdjust(Direction direction) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER_MANUALRESET : TURRET_POWER_MANUALRESET);
    }

    public void resetTurretEncoder() {
        stopTurret();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopTurret() {
        isTurretZero = true;
        turret.setPower(0);
    }

    public void stopTurretFloat() {
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stopTurret();
    }

    public void blueAutoOverride(boolean bao) {
        blueAutoOverride = bao;
    }

    public void resetTurretTicks() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public boolean isTurretZero() {
        return isTurretZero;
    }

    public double getCurrentDraw() {
        return turret.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public boolean isCurrentDrawPastThreshold() {
        return getCurrentDraw() > CURRENT_THRESHOLD;
    }

    public void lock() {
        lock.setPosition(0);
    }

    public void unlock() {
        lock.setPosition(1);
    }
}
