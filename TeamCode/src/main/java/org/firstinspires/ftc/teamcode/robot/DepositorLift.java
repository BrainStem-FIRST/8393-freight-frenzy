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
        DEFAULT, DEPLOY, ROTATECLEAR, EXTENDCLEAR, EXTENDCLEARACTION, EXTENDOUT, EXTENDOUTACTION,
                 RETRACT,             EXTENDBACK, EXTENDBACKACTION, TURRETRESET
    }

    public enum LiftGoal {
        DEFAULT, LIFTUP, LIFTUPACTION, LIFTDOWN, LIFTDOWNACTION, LIFTDOWNAUTOL1, LIFTUPAUTOL1
    }

    public enum DepositorHeight {
        LEVELONE, LEVELTWO, LEVELTHREE, CAP
    }

    private DcMotorEx lift;
    private DcMotorEx extend;
    private ServoImplEx gate;
    private ServoImplEx rotate;
    private RevTouchSensor touch;

    private static final double LIFT_UP_POWER = 1;
    private static final double LIFT_HOLD_POWER = 0.2;
    private static final double LIFT_STOP_POWER = 0;
    private static final double LIFT_UP_POWER_CAP = 0.7;
    private static final double LIFT_DOWN_POWER_CAP = -0.05;
    private static final double LIFT_DOWN_POWER = -0.8;

    private static final int LIFT_LEVELONE_TICKS = 175;
    private static final int LIFT_LEVELTWO_TICKS = 500;
    private static final int LIFT_RETURN_TICKS = 700;
    private static final int LIFT_LEVELTHREE_TICKS = 900;
    private static final int LIFT_CAP_TICKS = 1015;
    private int liftTicks = LIFT_LEVELTHREE_TICKS;

    private static final double EXTEND_OUT_POWER = 0.7;
    private static final double EXTEND_OUT_POWER_AUTO = 0.4;
    private static final double EXTEND_BACK_POWER = -0.8;
    private static final double EXTEND_BACK_POWER_MANUAL = -0.6;
    private static final double EXTEND_OUT_POWER_CAP = 0.4;
    private static final double EXTEND_BACK_POWER_CAP = -0.2;
    private static final double EXTEND_BACK_POWER_HOLD = -0.3;

    private static final int EXTEND_RESET_TICKS = 0;
    private static final int EXTEND_CLEAR_TICKS = 70;
    private static final int EXTEND_OUT_TICKS = 300;
    private static final int EXTEND_NORMAL_TICKS = 350;
    private static final int EXTEND_LEVELONE_TICKS = 1183;
    private static final int EXTEND_LEVELTWO_TICKS = 1260;
    private static final int EXTEND_LEVELTHREE_TICKS = 1320;
    private static final int EXTEND_CAP_TICKS = 1475;
    private int extendTicks = EXTEND_LEVELTHREE_TICKS;

    private static final double EXTEND_CURRENT_THRESHOLD = 7000;

    //x - deploys depositor a bit to clear

    private TimerCanceller rotateClearCanceller = new TimerCanceller(150);
    private TimerCanceller waitForLiftCanceller = new TimerCanceller(200);
    private TimerCanceller waitForExtendCanceller = new TimerCanceller(300);
    private TimerCanceller waitForGateCanceller = new TimerCanceller(300);
    private TimerCanceller extendBackCancellerTurret = new TimerCanceller(700);
    private TimerCanceller extendBackCancellerMotorTimeout = new TimerCanceller(1800);
    private TimerCanceller extendBackCancellerCurrentDrawTimeout = new TimerCanceller(750);
    private TimerCanceller turretTimeout;
    private TimerCanceller turretTimeoutOriginal = new TimerCanceller(1500);
    private TimerCanceller turretTimeoutStraight = new TimerCanceller(400);
    private TimerCanceller turretTimeoutInitial = new TimerCanceller(300);
    private TimerCanceller waitForLiftCancellerCap = new TimerCanceller(850);
    private TimerCanceller liftDownAutoL1Canceller = new TimerCanceller(100);
    private TimerCanceller liftUpAutoL1_2Canceller = new TimerCanceller(200);

    private TimerCanceller liftTimeout = new TimerCanceller(2000);

    private DepositorHeight depositHeight = DepositorHeight.LEVELTHREE;
    private DepositorGoal depositorGoal = DepositorGoal.DEFAULT;
    private LiftGoal liftGoal = LiftGoal.DEFAULT;
    private boolean hold = false;
    private boolean isAuto = false;
    private boolean extendStop = false;
    private boolean turbo = false;
    private Turret turret;
    private int i = 0;
    private int j = 0;

    public DepositorLift(HardwareMap map, Turret turret, boolean isAuto) {
        this.turret = turret;
        this.isAuto = isAuto;

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

        gate.setPwmRange(new PwmControl.PwmRange(960,1870));
        rotate.setPwmRange(new PwmControl.PwmRange(990,2405));
    }

    @Override
    public void reset() {
        rotateCollect();
        openCollect();
    }

    @Override
    public void update() {
        if (BrainSTEMRobot.mode == BrainSTEMRobot.Mode.SHARED
                || BrainSTEMRobot.mode ==  BrainSTEMRobot.Mode.STRAIGHT) {
            extendTicks = EXTEND_NORMAL_TICKS;
            liftTicks = LIFT_LEVELTWO_TICKS;
        } else if (BrainSTEMRobot.mode == BrainSTEMRobot.Mode.CAP) {
            extendTicks = EXTEND_OUT_TICKS;
            liftTicks = LIFT_LEVELTWO_TICKS;
        } else {
            setHeight(depositHeight);
        }
        if (BrainSTEMRobot.mode != BrainSTEMRobot.Mode.CAP) {
            switch (depositorGoal) {
                case DEFAULT:
                    break;
                case DEPLOY:
                    extendStop = false;
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
                    waitForExtendCanceller.reset();
                    setGoal(DepositorGoal.EXTENDOUTACTION);
                    break;
                case EXTENDOUTACTION:
                    if (waitForExtendCanceller.isConditionMet()) {
                        extend.setTargetPosition(extendTicks);
                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extend.setPower(EXTEND_OUT_POWER);
                        if (isAuto && depositHeight == DepositorHeight.LEVELONE) {
                            liftDownAutoL1Canceller.reset();
                            setGoal(LiftGoal.LIFTDOWNAUTOL1);
                        }
                        setGoal(DepositorGoal.DEFAULT);
                    }
                    break;
                case RETRACT:
                    //these two were timed from the beginning of the entire retract process
                    extendBackCancellerMotorTimeout.reset();
                    extendBackCancellerCurrentDrawTimeout.reset();
                    waitForGateCanceller.reset();
                    setGoal(DepositorGoal.EXTENDBACK);
                    break;
                case EXTENDBACK:
                    close();
                    if (waitForGateCanceller.isConditionMet()) {
                        rotateClear();
                        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        extendBackCancellerTurret.reset();
                        if (isAuto && depositHeight != DepositorHeight.LEVELTHREE) {
                            liftUpAutoL1_2Canceller.reset();
                        }
                        extend.setPower(EXTEND_BACK_POWER);
                        extendStop = true;
                        setGoal(DepositorGoal.EXTENDBACKACTION);
                    }
                    break;
                case EXTENDBACKACTION:
                    if (isAuto && depositHeight != DepositorHeight.LEVELTHREE & liftUpAutoL1_2Canceller.isConditionMet()) {
                        lift.setTargetPosition(LIFT_LEVELTHREE_TICKS);
                    }
                    if (extendBackCancellerTurret.isConditionMet()) {
                        turret.spinTurretReset();
                        if (BrainSTEMRobot.mode == BrainSTEMRobot.Mode.STRAIGHT) {
                            turretTimeout = turretTimeoutStraight;
                        } else {
                            turretTimeout = turretTimeoutOriginal;
                        }
                        turretTimeout.reset();
                        turretTimeoutInitial.reset();
                        setGoal(DepositorGoal.TURRETRESET);
                    }
                    break;
                case TURRETRESET:
                    if (turretTimeout.isConditionMet()
                            || (!turret.isTurretBusy() && turretTimeoutInitial.isConditionMet())) {
                        turret.stopTurret();
                        setGoal(LiftGoal.LIFTDOWN);
                        setGoal(DepositorGoal.DEFAULT);
                    }
                     break;
            }
            switch (liftGoal) {
                case DEFAULT:
                    break;
                case LIFTUP:
                    if (isAuto && depositHeight == DepositorHeight.LEVELONE) {
                        lift.setTargetPosition(LIFT_LEVELTWO_TICKS);
                    } else {
                        lift.setTargetPosition(liftTicks);
                    }
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    waitForLiftCanceller.reset();
                    setGoal(LiftGoal.LIFTUPACTION);
                    break;
                case LIFTUPACTION:
                    lift.setPower(LIFT_UP_POWER);
                    if (waitForLiftCanceller.isConditionMet()) {
                        if (BrainSTEMRobot.mode != BrainSTEMRobot.Mode.STRAIGHT) {
                            turret.spinTurretDeposit();
                        }
                        rotateDeposit();
                        if (isAuto) {
                            setGoal(DepositorGoal.EXTENDOUT);
                        }
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
                        extend.setPower(0);
                        lift.setPower(0);
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rotateCollect();
                        openCollect();
                        setGoal(LiftGoal.DEFAULT);
                    }
                    break;
                case LIFTDOWNAUTOL1:
                    if (liftDownAutoL1Canceller.isConditionMet()) {
                        lift.setTargetPosition(LIFT_LEVELONE_TICKS);
                        setGoal(LiftGoal.DEFAULT);
                    }
                    break;
            }
            if (extendStop &&
                    ((getExtendCurrentDraw() > EXTEND_CURRENT_THRESHOLD
                            && extendBackCancellerCurrentDrawTimeout.isConditionMet())
                        || extendBackCancellerMotorTimeout.isConditionMet())) {
                if (getExtendCurrentDraw() > EXTEND_CURRENT_THRESHOLD) {
                    Log.d("BrainSTEM", "passed threshold with value of " + getExtendCurrentDraw());
                }
                if (extendStop) {
                    Log.d("BrainSTEM", "extendStop true");
                }
                if (extendBackCancellerMotorTimeout.isConditionMet()) {
                    Log.d("BrainSTEM", "motor timeout condition met");
                }
                extend.setPower(0);
                extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extend.setPower(EXTEND_BACK_POWER_HOLD);
                extendStop = false;
            }
        } else {
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
                        setHeight(DepositorHeight.LEVELTWO);
                        setGoal(LiftGoal.LIFTUP);
                        setGoal(DepositorGoal.EXTENDCLEAR);
                    }
                    break;
                case EXTENDCLEAR:
                    extend.setTargetPosition(extendTicks);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(EXTEND_OUT_POWER_AUTO);
                    setGoal(DepositorGoal.DEFAULT);
                    break;
            }
            switch (liftGoal) {
                case DEFAULT:
                    break;
                case LIFTUP:
                    lift.setTargetPosition(liftTicks);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    waitForLiftCancellerCap.reset();
                    setGoal(LiftGoal.LIFTUPACTION);
                    break;
                case LIFTUPACTION:
                    lift.setPower(LIFT_UP_POWER);
                    if (waitForLiftCancellerCap.isConditionMet()) {
                        rotateDeposit();
                        setGoal(LiftGoal.DEFAULT);
                    }
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
        Log.d("BrainSTEM", "manual lift up");
//        close();
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(turbo ? LIFT_UP_POWER : LIFT_UP_POWER_CAP);
    }
    public void manualLiftHold() {
        if (hold) {
            lift.setPower(LIFT_HOLD_POWER);
        } else {
            lift.setPower(LIFT_STOP_POWER);
        }
    }
    public void manualLiftDown() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(LIFT_DOWN_POWER_CAP);
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
    public int getLiftTarget() {
        return lift.getTargetPosition();
    }

    //Extend
    public void manualExtendOutCap() {
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setPower(EXTEND_OUT_POWER_CAP);
    }
    public void manualExtendBack() {
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setPower(EXTEND_BACK_POWER_MANUAL);
    }
    public void manualExtendBackCap() {
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setPower(EXTEND_BACK_POWER_CAP);
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
    public double getExtendPower() {
        return extend.getPower();
    }
    public void resetExtendEncoder() {
        extend.setPower(0);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int getExtendTarget() {
        return extend.getTargetPosition();
    }

    //Gate
    public void close() {
        gate.setPosition(0);
    }
    public void openCap() {
        gate.setPosition(0.4);
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
        rotate.setPosition(0.1166);
    }
    public void rotateClear() {
        rotate.setPosition(0);
    }
    public void rotateDeposit() {
        rotate.setPosition(1);
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

    //Sensors
    public boolean isTouchPressed() {
        return touch.isPressed();
    }

    //Setters/Getters
    public void setHold(boolean isHolding) {
        hold = isHolding;
    }
}