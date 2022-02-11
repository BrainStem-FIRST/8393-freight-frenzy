package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        DEFAULT, DEPLOY, ROTATECLEAR, EXTENDCLEAR, EXTENDCLEARACTION, EXTENDOUT,
                 RETRACT,             EXTENDBACK, EXTENDBACKACTION
    }

    public enum LiftGoal {
        DEFAULT, LIFTUP, LIFTUPACTION, LIFTDOWNLEVELONE, LIFTDOWN, LIFTDOWNACTION
    }

    public enum DepositorHeight {
        LEVELONE, LEVELTWO, LEVELTHREE, CAP
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
    private static final double LIFT_DOWN_POWER = -0.7;

    private static final int LIFT_LEVELONE_TICKS = 270;
    private static final int LIFT_LEVELTWO_TICKS = 400;
    private static final int LIFT_LEVELTHREE_TICKS = 825;
    private static final int LIFT_CAP_TICKS = 1015;
    private int liftTicks = LIFT_LEVELTHREE_TICKS;

    private static final double EXTEND_OUT_POWER = 0.7;
    private static final double EXTEND_OUT_POWER_AUTO = 0.2;
    private static final double EXTEND_BACK_POWER = -1;

    private static final int EXTEND_RESET_TICKS = 0;
    private static final int EXTEND_CLEAR_TICKS = 70;
    private static final int EXTEND_LEVELONE_TICKS = 1183;
    private static final int EXTEND_LEVELTWO_TICKS = 1260;
    private static final int EXTEND_LEVELTHREE_TICKS = 1360;
    private static final int EXTEND_CAP_TICKS = 1475;
    private int extendTicks = EXTEND_LEVELTHREE_TICKS;

    private static final double EXTEND_CURRENT_THRESHOLD = 5500;

    private TimerCanceller rotateClearCanceller = new TimerCanceller(150);
    private TimerCanceller waitForLiftCanceller = new TimerCanceller(400);
    private TimerCanceller waitForExtendCanceller = new TimerCanceller(500);
    private TimerCanceller waitForGateCanceller = new TimerCanceller(300);
    private TimerCanceller extendBackCancellerTurret = new TimerCanceller(400);
    private TimerCanceller extendBackCancellerMotorTimeout = new TimerCanceller(750);
//    private TimerCanceller extendBackCancellerTurretTimeout

    private TimerCanceller liftTimeout = new TimerCanceller(2000);

    private DepositorHeight depositHeight = DepositorHeight.LEVELTHREE;
    private DepositorGoal depositorGoal = DepositorGoal.DEFAULT;
    private LiftGoal liftGoal = LiftGoal.DEFAULT;
    private boolean hold = false;
    private boolean cap = false;
    private boolean auto = false;
    private Mode mode = Mode.ANGLED;

    private Turret turret;
    private int i = 0;
    private int j = 0;

    public DepositorLift(HardwareMap map, Turret turret) {
        this.turret = turret;
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        extend = new CachingMotor(map.get(DcMotorEx.class, "extend"));
        gate = new CachingServo(map.get(ServoImplEx.class, "depositorGate"));
        rotate = new CachingServo(map.get(ServoImplEx.class, "rotate"));
        touch = map.get(RevTouchSensor.class, "liftTouch");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setTargetPositionTolerance(3);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setTargetPositionTolerance(3);

        gate.setPwmRange(new PwmControl.PwmRange(1000,1870));
        rotate.setPwmRange(new PwmControl.PwmRange(1050,2465));
    }

    @Override
    public void reset() {
        rotateCollect();
        openCollect();
    }

    @Override
    public void update() {
        if (!cap) {
            switch (depositorGoal) {
                case DEFAULT:
                    break;
                case DEPLOY:
                    close();
                    rotateClearCanceller.reset();
                    setGoal(DepositorGoal.ROTATECLEAR);
                    break;
                case ROTATECLEAR:
                    rotateClear();
                    if (rotateClearCanceller.isConditionMet()) {
                        setHeight(depositHeight);
                        setGoal(LiftGoal.LIFTUP);
                        setGoal(DepositorGoal.EXTENDCLEAR);
                    }
                    break;
                case EXTENDCLEAR:
                    extend.setTargetPosition(EXTEND_CLEAR_TICKS);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(EXTEND_OUT_POWER_AUTO);
                    setGoal(DepositorGoal.DEFAULT);
                    break;
                case EXTENDOUT:
                    if (waitForExtendCanceller.isConditionMet()) {
                        extend.setTargetPosition(extendTicks);
                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extend.setPower(auto ? EXTEND_OUT_POWER_AUTO : EXTEND_OUT_POWER);
                        rotateDeposit();
                        setGoal(DepositorGoal.DEFAULT);
                    }
                    break;
                case RETRACT:
                    extendBackCancellerTurret.reset();
                    waitForGateCanceller.reset();
                    setGoal(DepositorGoal.EXTENDBACK);
                    break;
                case EXTENDBACK:
                    close();
                    if (waitForGateCanceller.isConditionMet()) {
                        rotateClear();
                        extend.setTargetPosition(EXTEND_RESET_TICKS);
                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendBackCancellerTurret.reset();
                        extendBackCancellerMotorTimeout.reset();
                        setGoal(DepositorGoal.EXTENDBACKACTION);
                    }
                    break;
                case EXTENDBACKACTION:
                    extend.setPower(EXTEND_BACK_POWER);
                    if (getExtendCurrentDraw() > EXTEND_CURRENT_THRESHOLD || extendBackCancellerMotorTimeout.isConditionMet()) {
                        extend.setPower(0);
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if (extendBackCancellerTurret.isConditionMet()) {
                        if (mode != Mode.STRAIGHT) {
                            turret.spinTurretReset();
                        }
                        if (!turret.isTurretBusy() && Math.abs(turret.encoderPosition()) < 5) {
                            turret.stopTurret();
                            setGoal(LiftGoal.LIFTDOWN);
                            setGoal(DepositorGoal.DEFAULT);
                        }
                    }
                    break;
            }
            switch (liftGoal) {
                case DEFAULT:
                    break;
                case LIFTUP:
                    lift.setTargetPosition(liftTicks);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    waitForLiftCanceller.reset();
                    setGoal(LiftGoal.LIFTUPACTION);
                    break;
                case LIFTUPACTION:
                    lift.setPower(LIFT_UP_POWER);
                    if (waitForLiftCanceller.isConditionMet()) {
                        if (mode != Mode.STRAIGHT) {
                            turret.spinTurretDeposit();
                        }
                        waitForExtendCanceller.reset();
                        setGoal(DepositorGoal.EXTENDOUT);
                        setGoal(LiftGoal.DEFAULT);
                    }
                    break;
                case LIFTDOWN:
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftTimeout.reset();
                    setGoal(LiftGoal.LIFTDOWNACTION);
                    break;
                case LIFTDOWNACTION:
                    lift.setPower(LIFT_DOWN_POWER);
                    if (touch.isPressed() || liftTimeout.isConditionMet()) {
                        lift.setPower(0);
                        rotateCollect();
                        openCollect();
                        setGoal(LiftGoal.DEFAULT);
                        break;
                    }
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
    public double getExtendCurrentDraw() {
        return extend.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public void adjustExtend(int ticks) {
        extendTicks += ticks;
        extend.setTargetPosition(extendTicks);
    }

    //Gate
    public void close() {
        gate.setPosition(0);
    }
    public void open() {
        switch (depositHeight) {
            case LEVELONE:
            case LEVELTWO:
                openPartial();
                break;
            case LEVELTHREE:
                openFull();
                break;
        }
    }
    public void openCollect() {
        gate.setPosition(0.724);
    }
    public void openPartial() {
        gate.setPosition(0.44);
    }
    public void openFull() {
        gate.setPosition(1);
    }
    public void test(double position) {
        gate.setPosition(position);
    }

    //Rotate
    public void rotateCollect() {
//        Log.d("DepositorLift", "rotateCollect");
        rotate.setPosition(0.1166);
    }
    public void rotateClear() {
//        Log.d("DepositorLift", "rotateClear");
        rotate.setPosition(0);
    }
    public void rotateDeposit() {
//        Log.d("DepositorLift", "rotateDeposit f");
        switch (depositHeight) {
            case LEVELONE:
//                rotate.setPosition(0.8);
//                break;
            case LEVELTWO:
            case LEVELTHREE:
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
            case LEVELONE:
                liftTicks = LIFT_LEVELONE_TICKS;
                extendTicks = EXTEND_LEVELONE_TICKS;
                break;
            case LEVELTWO:
                liftTicks = LIFT_LEVELTWO_TICKS;
                extendTicks = EXTEND_LEVELTWO_TICKS;
                break;
            case LEVELTHREE:
                liftTicks = LIFT_LEVELTHREE_TICKS;
                extendTicks = EXTEND_LEVELTHREE_TICKS;
                break;
            case CAP:
                liftTicks = LIFT_CAP_TICKS;
                extendTicks = EXTEND_CAP_TICKS;
                break;
        }
    }

    public int getLiftTarget() {
        return lift.getTargetPosition();
    }

    public int getExtendTarget() {
        return extend.getTargetPosition();
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
    public void setAuto(boolean isAuto) {
        auto = isAuto;
    }
}