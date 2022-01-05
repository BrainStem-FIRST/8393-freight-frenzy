package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class DepositorLift implements Component {
    public enum Goal {
        DEFAULT, DEPLOY, DEPLOYACTION, RETRACT, RETRACTACTION
    }

    private DcMotorEx lift;
    private ServoImplEx gate;
    private ServoImplEx flip;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;
    private DigitalChannel limit;

    private static final double LIFT_UP_POWER = 0.85; //0.85
    private static final double LIFT_DOWN_POWER = -0.7; //0.4
    private static final int LIFT_LEVELONE_TICKS = 0;
    private static final int LIFT_LEVELTWO_TICKS = 170;
    private static final int LIFT_LEVELTHREE_TICKS = 350;
    private int liftTicks = LIFT_LEVELTHREE_TICKS;

    private TimerCanceller liftCanceller = new TimerCanceller(2000);
    private TimerCanceller flipCanceller = new TimerCanceller(200);

    private BarcodePattern depositHeight = BarcodePattern.LEVELTHREE;
    private Goal goal = Goal.DEFAULT;

    public DepositorLift(HardwareMap map, Telemetry telemetry) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        gate = new CachingServo(map.get(ServoImplEx.class, "depositorGate"));
        flip = new CachingServo(map.get(ServoImplEx.class, "flip"));
        extend = new CachingServo(map.get(ServoImplEx.class, "extend"));
        shippingElementGrab = new CachingServo(map.get(ServoImplEx.class, "SEGrab"));
        limit = map.digitalChannel.get("liftLimit");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));

        gate.setPwmRange(new PwmControl.PwmRange(1200,2300));
        flip.setPwmRange(new PwmControl.PwmRange(660,1920));
        extend.setPwmRange(new PwmControl.PwmRange(720,1460));
        shippingElementGrab.setPwmRange(new PwmControl.PwmRange(870,1920));
    }

    @Override
    public void reset() {
        autoLiftDown();
        open();
        retract();
        flipMid();
        releaseSE();
    }

    @Override
    public void update() {
        switch(goal) {
            case DEFAULT:
                break;
            case DEPLOY:
                flipCanceller.reset();
                close();
                setGoal(Goal.DEPLOYACTION);
                break;
            case DEPLOYACTION:
                flipOut();
                if (flipCanceller.isConditionMet()) {
                    extend();
                }
                break;
            case RETRACT:
                flipCanceller.reset();
                openFull();
                setGoal(Goal.RETRACTACTION);
                break;
            case RETRACTACTION:
                flipIn();
                if (flipCanceller.isConditionMet()) {
                    retract();
                }
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }

    public void autoLiftUp() {
        if (depositHeight != BarcodePattern.LEVELONE) {
            close();
            lift.setTargetPosition(liftTicks);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(LIFT_UP_POWER);
            while (lift.isBusy());
//            holdLift();
//            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void autoLiftDown() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftCanceller.reset();
        while(!limit.getState() && !liftCanceller.isConditionMet()) {
            lift.setPower(LIFT_DOWN_POWER);
        }
        stopLift();
    }

    public void manualLiftUp() {
        close();
        lift.setPower(LIFT_UP_POWER);
    }

    public void manualLiftDown() {
        lift.setPower(LIFT_DOWN_POWER);
    }

    public void stopLift() {
        lift.setPower(0);
    }

    public Goal getGoal() {
        return goal;
    }

    public void setGoal(Goal goal) {
        if (this.goal != goal)
        {
            liftCanceller.reset();
            this.goal = goal;
        }
    }

    public void setHeight(BarcodePattern height) {
        depositHeight = height;
        switch(height) {
            case LEVELONE:
                liftTicks = LIFT_LEVELONE_TICKS;
                break;
            case LEVELTWO:
                liftTicks = LIFT_LEVELTWO_TICKS;
                break;
            case LEVELTHREE:
                liftTicks = LIFT_LEVELTHREE_TICKS;
                break;
        }
    }

    public BarcodePattern getHeight() {
        return depositHeight;
    }

    public void close() {
        gate.setPosition(0);
    }

    public void open() {
        if (depositHeight == BarcodePattern.LEVELONE) {
            gate.setPosition(0.4);
        } else {
            openFull();
        }
    }

    public void openFull() {
        gate.setPosition(1);
    }

    public void extend() {
        if (depositHeight == BarcodePattern.LEVELONE) {
            extend.setPosition(0.21);
        } else {
            extend.setPosition(0);
        }
    }

    public void retract() {
        extend.setPosition(1);
    }

    public void flipOut() {
        flip.setPosition(1);
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

    public int getLiftTicks() {
        return lift.getCurrentPosition();
    }

    public boolean limitState() {
        return limit.getState();
    }
}