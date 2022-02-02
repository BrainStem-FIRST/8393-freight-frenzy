package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class DepositorLift implements Component {
    /*
    Out: isDeploying = true
    DepositorGoal: DEPLOY (called) -> ROTATECLEAR (rotate servo for extend clearance) -> EXTENDOUT -> EXTENDOUTACTION (extend out LEVEL DEPENDENT) -> ROTATEDEPOSIT (rotate servo LEVEL DEPENDENT)
    LiftGoal:                                                                         -> LIFTUP (LEVEL DEPENDENT) -> LIFTUPACTION
    TurretGoal:                                                                                     ||-> ROTATE (POSITION DEPENDENT)
     */

    /*
    In: isDeploying = false
    DepositorGoal: RETRACT (called) -> EXTENDBACK -> EXTENDBACKACTION (extend back) -> ROTATECLEAR (rotate servo for extend clearance)    ROTATECOLLECT (rotate servo)
    LiftGoal:                                             ||    -> LIFTDOWN (LEVEL DEPENDENT) -----------------------------------------------^> LIFTDOWNACTION
    TurretGoal:                                           ||    -> ROTATERESET
     */

    public enum DepositorGoal {
        DEFAULT, DEPLOY, ROTATECLEAR, EXTENDOUT,  EXTENDOUTACTION,  ROTATEDEPOSIT,
                 RETRACT,             EXTENDBACK, EXTENDBACKACTION, ROTATECOLLECT
    }

    public enum LiftGoal {
        STOP, LIFTUP, LIFTUPACTION, LIFTDOWN, LIFTDOWNACTION
    }

    public enum DepositorHeight {
        LOW, MIDDLE, HIGH, CAP
    }

    public enum Mode {
        ANGLED, STRAIGHT, CAP
    }

    private DcMotorEx lift;
    private DcMotorEx extend;
    private ServoImplEx gate;
    private ServoImplEx rotate;
    private RevTouchSensor touch;

    private static final double LIFT_UP_POWER = 1;
    private static final double LIFT_HOLD_POWER = 0.2;
    private static final double LIFT_STOP_POWER = 0;
    private static final double LIFT_DOWN_POWER_SLOW = -0.05;
    private static final double LIFT_DOWN_POWER = -0.4;

    private static final int LIFT_LEVELONE_TICKS = 85;
    private static final int LIFT_LEVELTWO_TICKS = 339;
    private static final int LIFT_LEVELTHREE_TICKS = 718;
    private static final int LIFT_CAP_TICKS = 1015;
    private int liftTicks = LIFT_LEVELTHREE_TICKS;

    private static final double EXTEND_OUT_POWER = 0.5;
    private static final double EXTEND_BACK_POWER = -0.5;

    private static final int EXTEND_RESET_TICKS = 0;
    private static final int EXTEND_LEVELONE_TICKS = 0;
    private static final int EXTEND_LEVELTWO_TICKS = 0;
    private static final int EXTEND_LEVELTHREE_TICKS = 0;
    private static final int EXTEND_CAP_TICKS = 0;
    private int extendTicks = EXTEND_LEVELTHREE_TICKS;

    private TimerCanceller rotateClearCanceller = new TimerCanceller(150);
    private TimerCanceller extendWaitCanceller = new TimerCanceller(300);
    private TimerCanceller extendOutCanceller = new TimerCanceller(600);
    private TimerCanceller extendBackCanceller = new TimerCanceller(200);
    private TimerCanceller rotateDepositCanceller = new TimerCanceller(500);
    private TimerCanceller rotateCollectCanceller = new TimerCanceller(150);

    private TimerCanceller liftTimeout = new TimerCanceller(2000);

    private DepositorHeight depositHeight = DepositorHeight.HIGH;
    private DepositorGoal depositorGoal = DepositorGoal.DEFAULT;
    private LiftGoal liftGoal = LiftGoal.STOP;
    private boolean isDeploying = true;
    private boolean hold = false;
    private boolean cap = false;
    private Mode mode = Mode.ANGLED;

    public DepositorLift(HardwareMap map) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        extend = new CachingMotor(map.get(DcMotorEx.class, "extend"));
        gate = new CachingServo(map.get(ServoImplEx.class, "depositorGate"));
        rotate = new CachingServo(map.get(ServoImplEx.class, "rotate"));
        touch = map.get(RevTouchSensor.class, "liftTouch");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPositionTolerance(3);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setTargetPositionTolerance(3);

//        gate.setPwmRange(new PwmControl.PwmRange(1200,2300));
        rotate.setPwmRange(new PwmControl.PwmRange(806,2240));
        //806 - clear
        //1000 - collect 0.1352859135
        //2240 - deposit
    }

    @Override
    public void reset() {
        rotateCollect();
        open();
    }

    @Override
    public void update() {
        switch(depositorGoal) {
            case DEFAULT:
                break;
            case DEPLOY:
                isDeploying = true;
                close();
                rotateClearCanceller.reset();
                setGoal(DepositorGoal.ROTATECLEAR);
                break;
            case ROTATECLEAR:
                rotateClear();
                if (rotateClearCanceller.isConditionMet()) {
                    if (isDeploying) {
                        extendWaitCanceller.reset();
                        setGoal(DepositorGoal.EXTENDOUT);
                        setGoal(LiftGoal.LIFTUP);
                    } else {
                        setGoal(DepositorGoal.DEFAULT);
                    }
                }
                break;
            case EXTENDOUT:
                extend.setTargetPosition(extendTicks);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(!extendWaitCanceller.isConditionMet()) {
                    extendOutCanceller.reset();
                    setGoal(DepositorGoal.EXTENDOUTACTION);
                }
                break;
            case EXTENDOUTACTION:
                extend.setPower(EXTEND_OUT_POWER);
                if (extendOutCanceller.isConditionMet()) {
                    rotateDepositCanceller.reset();
                    setGoal(DepositorGoal.ROTATEDEPOSIT);
                }
                break;
            case ROTATEDEPOSIT:
                rotateDeposit();
                if (rotateDepositCanceller.isConditionMet()) {
                    setGoal(DepositorGoal.DEFAULT);
                }
                break;
            case RETRACT:
                isDeploying = false;
                extendBackCanceller.reset();
                setGoal(DepositorGoal.EXTENDBACK);
                break;
            case EXTENDBACK:
                extend.setTargetPosition(EXTEND_RESET_TICKS);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendBackCanceller.reset();
                setGoal(DepositorGoal.EXTENDBACKACTION);
                break;
            case EXTENDBACKACTION:
                extend.setPower(EXTEND_BACK_POWER);
                if (extendBackCanceller.isConditionMet()) {
                    rotateClearCanceller.reset();
                    setGoal(DepositorGoal.ROTATECLEAR);
                    if (mode != Mode.STRAIGHT) {
                        //setTurretGoal reset
                    }
                    setGoal(LiftGoal.LIFTDOWN);
                }
                break;
            case ROTATECOLLECT:
                rotateCollect();
                if (rotateCollectCanceller.isConditionMet()) {
                    setGoal(DepositorGoal.DEFAULT);
                }
                break;
        }
        switch(liftGoal) {
            case LIFTUP:
                lift.setTargetPosition(liftTicks);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (mode != Mode.STRAIGHT) {
                    //setTurretGoal rotate
                }
                setGoal(LiftGoal.LIFTUPACTION);
                break;
            case LIFTUPACTION:
                lift.setPower(LIFT_UP_POWER);
                break;
            case LIFTDOWN:
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftTimeout.reset();
                setGoal(LiftGoal.LIFTDOWNACTION);
                break;
            case LIFTDOWNACTION:
                lift.setPower(LIFT_DOWN_POWER);
                if(touch.isPressed() || liftTimeout.isConditionMet()) {
                    lift.setPower(0);
                    openFull();
                    rotateCollectCanceller.reset();
                    setGoal(DepositorGoal.ROTATECOLLECT);
                    setGoal(LiftGoal.STOP);
                    break;
                }
        }
    }

    @Override
    public String test() {
        return null;
    }

    //Lift
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
    public double getLiftPower() {
        return lift.getPower();
    }
    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    //Extend
    public void manualExtendOut() {
        extend.setPower(EXTEND_OUT_POWER);
    }
    public void manualExtendBack() {
        extend.setPower(EXTEND_BACK_POWER);
    }
    public void stopExtend() {
        extend.setPower(0);
    }
    public int getExtendPosition() {
        return extend.getCurrentPosition();
    }

    //Gate
    public void close() {
        gate.setPosition(0);
    }
    public void open() {
        switch (depositHeight) {
            case LOW:
            case MIDDLE:
                openPartial();
                break;
            case HIGH:
                openFull();
                break;
        }
    }
    public void openPartial() {
        gate.setPosition(0.4);
    }
    public void openFull() {
        gate.setPosition(1);
    }
    public void test(double position) {
        gate.setPosition(position);
    }

    //Rotate
    public void rotateCollect() {
        rotate.setPosition(0.1352859135);
    }
    public void rotateClear() {
        rotate.setPosition(1);
    }
    public void rotateDeposit() {
        switch (depositHeight) {
            case LOW:
//                rotate.setPosition(0.8);
//                break;
            case MIDDLE:
            case HIGH:
                rotate.setPosition(1);
                break;
        }
    }

    //LiftGoal
    public LiftGoal getLiftGoal() {
        return liftGoal;
    }
    public void setGoal(LiftGoal liftGoal) {
        if (this.liftGoal != liftGoal) {
            this.liftGoal = liftGoal;
        }
    }

    //DepositorGoal
    public DepositorGoal getDepositorGoal() {
        return depositorGoal;
    }
    public void setGoal(DepositorGoal depositorGoal) {
        if (this.depositorGoal != depositorGoal) {
            this.depositorGoal = depositorGoal;
        }
    }

    //DepositorHeight
    public DepositorHeight getHeight() {
        return depositHeight;
    }

    public void setHeight(DepositorHeight height) {
        depositHeight = height;
        switch(height) {
            case LOW:
                liftTicks = LIFT_LEVELONE_TICKS;
                extendTicks = EXTEND_LEVELONE_TICKS;
                break;
            case MIDDLE:
                liftTicks = LIFT_LEVELTWO_TICKS;
                extendTicks = EXTEND_LEVELTWO_TICKS;
                break;
            case HIGH:
                liftTicks = LIFT_LEVELTHREE_TICKS;
                extendTicks = EXTEND_LEVELTHREE_TICKS;
                break;
            case CAP:
                liftTicks = LIFT_CAP_TICKS;
                extendTicks = EXTEND_CAP_TICKS;
                break;
        }
    }

    //Touch
    public boolean isTouchPressed() {
        return touch.isPressed();
    }

    //Setters
    public void setHold(boolean isHolding) {
        hold = isHolding;
    }
    public void setCap(boolean isCapping) {
        cap = isCapping;
    }
}