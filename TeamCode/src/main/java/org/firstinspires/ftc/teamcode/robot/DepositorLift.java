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
        LOW, MIDDLE, HIGH
    }

    private DcMotorEx lift;
    private ServoImplEx gate;
    private ServoImplEx flip;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;

    private static final double LIFT_UP_POWER = 0.4; //0.85
    private static final double LIFT_DOWN_POWER = -0.2; //0.4
    private static final int LIFT_TIMEOUT = 3000;

    private TimerCanceller liftCanceller = new TimerCanceller(LIFT_TIMEOUT);
    private TimerCanceller flipCanceller = new TimerCanceller(200);

    private Height depositHeight = Height.HIGH;
    private Goal goal = Goal.STOP;
    private boolean depositFirstLevel = false;

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

        gate.setPwmRange(new PwmControl.PwmRange(800,1850));
        flip.setPwmRange(new PwmControl.PwmRange(930,2140));
        extend.setPwmRange(new PwmControl.PwmRange(1700,2360));
        shippingElementGrab.setPwmRange(new PwmControl.PwmRange(1530,2520));
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
                setGoal(Goal.DEPLOYACTION);
                break;
            case DEPLOYACTION:
                close();
                flipOut();
                if (flipCanceller.isConditionMet()) {
                    if (depositFirstLevel) {
                        extendFirstLevel();
                    } else {
                        extend();
                    }
                }
                break;
            case RETRACT:
                flipCanceller.reset();
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
        lift.setPower(LIFT_UP_POWER);
    }

    public void liftDown() {
        lift.setPower(LIFT_DOWN_POWER);
    }

    public void stopLift() {
        lift.setPower(0);
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
        gate.setPosition(1);
    }

    public void extend() {
        extend.setPosition(0);
    }

    public void extendFirstLevel() {
        extend.setPosition(0.21);
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

    public void setDepositorFirstLevel(boolean dFL) {
        depositFirstLevel = dFL;
    }

    public boolean getDepositFirstLevel() {
        return depositFirstLevel;
    }
}