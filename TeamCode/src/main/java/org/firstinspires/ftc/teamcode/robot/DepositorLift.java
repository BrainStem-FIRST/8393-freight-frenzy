package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class DepositorLift implements Component {
    public enum DepositorGoal {
        DEFAULT, DEPLOY, DEPLOYACTION, RETRACT, RETRACTACTION
    }

    public enum LiftGoal {
        STOP, LIFTUP, LIFTUPACTION, LIFTDOWN, LIFTDOWNACTION
    }

    public enum DepositorHeight {
        LOW, MIDDLE, HIGH, CAP
    }

    private DcMotorEx lift;
    private ServoImplEx gate;
    private ServoImplEx flip;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;
    private RevTouchSensor touch;

    private static final double LIFT_UP_POWER = 1;
    private static final double LIFT_HOLD_POWER = 0.2;
    private static final double LIFT_STOP_POWER = 0;
    private static final double LIFT_DOWN_POWER_SLOW = -0.05;
    private static final double LIFT_DOWN_POWER = -0.4;
    private static final int LIFT_LEVELONE_TICKS = 0;
    private static final int LIFT_LEVELTWO_TICKS = 170;
    private static final int LIFT_LEVELTHREE_TICKS = 400;
    private static final int LIFT_CAP_TICKS = 400;
    private int liftTicks = LIFT_LEVELTHREE_TICKS;

    private TimerCanceller liftCanceller = new TimerCanceller(2000);
    private TimerCanceller flipCanceller = new TimerCanceller(200);

    private DepositorHeight depositHeight = DepositorHeight.HIGH;
    private DepositorGoal depositorGoal = DepositorGoal.DEFAULT;
    private LiftGoal liftGoal = LiftGoal.STOP;
    private boolean hold = false;
    private boolean cap = false;
    private boolean autoSE = false;

    public DepositorLift(HardwareMap map, Telemetry telemetry) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        gate = new CachingServo(map.get(ServoImplEx.class, "depositorGate"));
        flip = new CachingServo(map.get(ServoImplEx.class, "flip"));
        extend = new CachingServo(map.get(ServoImplEx.class, "extend"));
        shippingElementGrab = new CachingServo(map.get(ServoImplEx.class, "SEGrab"));
        touch = map.get(RevTouchSensor.class, "liftTouch");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPositionTolerance(3);
//        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));

        gate.setPwmRange(new PwmControl.PwmRange(1200,2300));
        flip.setPwmRange(new PwmControl.PwmRange(660,1920));
        extend.setPwmRange(new PwmControl.PwmRange(720,1460));
        shippingElementGrab.setPwmRange(new PwmControl.PwmRange(1040,1840));
    }

    @Override
    public void reset() {
        flipIn();
        open();
        retract();
        releaseSE();
    }

    @Override
    public void update() {
        switch(depositorGoal) {
            case DEPLOY:
                flipCanceller.reset();
                close();
                setGoal(DepositorGoal.DEPLOYACTION);
                break;
            case DEPLOYACTION:
                if (cap) {
                    flipCap();
                } else if (autoSE) {
                    flipAutoSE();
                } else {
                    flipOut();
                }
                if (flipCanceller.isConditionMet()) {
                    extend();
                }
                break;
            case RETRACT:
                flipCanceller.reset();
                openFull();
                setGoal(DepositorGoal.RETRACTACTION);
                break;
            case RETRACTACTION:
                flipIn();
                if (flipCanceller.isConditionMet()) {
                    retract();
                }
                break;
        }
        switch(liftGoal) {
            case LIFTUP:
                close();
                if (depositHeight == DepositorHeight.LOW) {
                    break;
                }
                lift.setTargetPosition(liftTicks);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftCanceller.reset();
                setGoal(LiftGoal.LIFTUPACTION);
                break;
            case LIFTUPACTION:
                lift.setPower(LIFT_UP_POWER);
//                    if (liftCanceller.isConditionMet()) {
//                        setGoal(LiftGoal.STOP);
//                    }
                break;
            case LIFTDOWN:
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftCanceller.reset();
                setGoal(LiftGoal.LIFTDOWNACTION);
                break;
            case LIFTDOWNACTION:
                lift.setPower(LIFT_DOWN_POWER);
                if(touch.isPressed() || liftCanceller.isConditionMet()) {
                    lift.setPower(0);
                    openFull();
                    setGoal(LiftGoal.STOP);
                    break;
                }
        }
    }

    @Override
    public String test() {
        return null;
    }

    public void manualLiftUp() {
        close();
        lift.setPower(LIFT_UP_POWER);
    }

    public void manualLiftHold() {
        if (hold) {
            lift.setPower(LIFT_HOLD_POWER);
        } else {
            lift.setPower(LIFT_STOP_POWER);
        }
    }

    public void slowLiftDown() {
        lift.setPower(LIFT_DOWN_POWER_SLOW);
    }

    public void manualLiftDown() {
        lift.setPower(LIFT_DOWN_POWER);
    }

    public void stopLift() {
        lift.setPower(0);
    }

    public LiftGoal getLiftGoal() {
        return liftGoal;
    }

    public void setGoal(LiftGoal liftGoal) {
        if (this.liftGoal != liftGoal)
        {
            this.liftGoal = liftGoal;
        }
    }

    public DepositorGoal getDepositorGoal() {
        return depositorGoal;
    }

    public void setGoal(DepositorGoal depositorGoal) {
        if (this.depositorGoal != depositorGoal)
        {
            this.depositorGoal = depositorGoal;
        }
    }

    public void setHeight(DepositorHeight height) {
        depositHeight = height;
        switch(height) {
            case LOW:
                liftTicks = LIFT_LEVELONE_TICKS;
                break;
            case MIDDLE:
                liftTicks = LIFT_LEVELTWO_TICKS;
                break;
            case HIGH:
                liftTicks = LIFT_LEVELTHREE_TICKS;
                break;
            case CAP:
                liftTicks = LIFT_CAP_TICKS;
                break;
        }
    }

    public DepositorHeight getHeight() {
        return depositHeight;
    }

    public void close() {
        gate.setPosition(0);
    }

    public void open() {
        if (depositHeight == DepositorHeight.HIGH) {
            openFull();
        } else {
            openPartial();
        }
    }

    public void openFull() {
        gate.setPosition(1);
    }

    public void openPartial() {
        gate.setPosition(0.4);
    }

    public void extend() {
        if (depositHeight == DepositorHeight.LOW) {
            extend.setPosition(0.48);
        } else {
            extend.setPosition(0);
        }
    }

    public void extendSE() {
        for (int i = 0; i < 5; i++) {
            extend.setPosition(extend.getPosition() - 0.05);
        }
    }

    public void retract() {
        extend.setPosition(1);
    }

    public void flipOut() {
        flip.setPosition(1);
    }

    public void flipAutoSE() {
        flip.setPosition(0.9);
    }

    public void flipCap() {
        flip.setPosition(0.7976190476);
    }

    public void flipMid() {
        flip.setPosition(0.2857);
    }

    public void flipIn() {
        flip.setPosition(0);
    }

    public void clampSE() {
        shippingElementGrab.setPosition(0);
    }

    public void releaseSE() {
        shippingElementGrab.setPosition(1);
    }

    public double getSEPosition() {
        return shippingElementGrab.getPosition();
    }

    public int getLiftTicks() {
        return lift.getCurrentPosition();
    }

    public boolean isTouchPressed() {
        return touch.isPressed();
    }

    public void setHold(boolean isHolding) {
        hold = isHolding;
    }

    public void setCap(boolean isCapping) {
        cap = isCapping;
    }

    public void setAutoSE(boolean isAutoSE) {
        autoSE = isAutoSE;
    }

    public double getLiftPower() {
        return lift.getPower();
    }
}