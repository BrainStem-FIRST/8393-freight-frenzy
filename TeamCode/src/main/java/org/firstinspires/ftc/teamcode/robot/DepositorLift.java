package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class DepositorLift implements Component {
    public enum Goal {
        UP, DOWN, STOP, DEPLOY, DEPLOYACTION, RETRACT, RETRACTACTION
    }

    public enum Height {
        LEVELONE, LEVELTWO, LEVELTHREE
    }

    private DcMotorEx lift;
    private ServoImplEx gate;
    private ServoImplEx flip;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;

    private static final double LIFT_UP_POWER = 0.4; //0.85
    private static final double LIFT_DOWN_POWER = -0.2; //0.4
    private static final double LIFT_LEVELTWO_TICKS = 0;
    private static final double LIFT_LEVELTHREE_TICKS = 0;

    private TimerCanceller liftCanceller = new TimerCanceller(3000);
    private TimerCanceller flipCanceller = new TimerCanceller(200);

    private Height depositHeight = Height.LEVELTHREE;
    private Goal goal = Goal.STOP;
    private boolean depositLow = false;

    public DepositorLift(HardwareMap map, Telemetry telemetry) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        gate = new CachingServo(map.get(ServoImplEx.class, "depositorGate"));
        flip = new CachingServo(map.get(ServoImplEx.class, "flip"));
        extend = new CachingServo(map.get(ServoImplEx.class, "extend"));
        shippingElementGrab = new CachingServo(map.get(ServoImplEx.class, "SEGrab"));

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setPwmRange(new PwmControl.PwmRange(1200,2380));
        flip.setPwmRange(new PwmControl.PwmRange(660,1920));
        extend.setPwmRange(new PwmControl.PwmRange(720,1460));
        shippingElementGrab.setPwmRange(new PwmControl.PwmRange(870,1920));
    }

    @Override
    public void reset() {
        open();
        retract();
        flipIn();
        releaseSE();
    }

    @Override
    public void update() {
        switch(goal) {
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

    public void liftUp() {
        close();
        lift.setPower(-LIFT_UP_POWER);
    }

    public void liftDown() {
        lift.setPower(-LIFT_DOWN_POWER);
    }

    public void stopLift() {
        lift.setPower(0.0);
    }

    public Goal getGoal()
    {
        return goal;
    }

    public void setGoal(Goal goal)
    {
        if (this.goal != goal)
        {
            liftCanceller.reset();
            this.goal = goal;
        }
    }

    public void setHeight(Height height) {
        depositHeight = height;
    }

    public void close() {
        gate.setPosition(0);
    }

    public void open() {
        if (depositLow) {
            gate.setPosition(0.4);
        } else {
            openFull();
        }
    }

    public void openFull() {
        gate.setPosition(1);
    }

    public void extend() {
        if (depositLow) {
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

    public void flipIn() {
        flip.setPosition(0);
    }

    public void clampSE() {
        shippingElementGrab.setPosition(0);
    }

    public void releaseSE() {
        shippingElementGrab.setPosition(1);
    }

    public void setDepositLow(boolean dL) {
        depositLow = dL;
    }

    public boolean getDepositLow() {
        return depositLow;
    }
}