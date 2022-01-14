package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

import java.util.Timer;

public class Collector implements Component {
    public enum Goal {
        DEFAULT, DEPLOY, DEPLOYACTION, RETRACT, RETRACTACTION, OFF, OPEN, OFFAUTO
    }
    private DcMotorEx collector;
    private ServoImplEx tilt;
    private ServoImplEx gate;

    private static final double COLLECT_POWER = 1;
    private int sign = 1;
    private Goal goal = Goal.DEFAULT;
    private boolean gateOverride = false;
    private TimerCanceller deployCanceller = new TimerCanceller(75);
    private TimerCanceller retractCanceller = new TimerCanceller(400);
    private TimerCanceller offCanceller = new TimerCanceller(700);
    private TimerCanceller gateCanceller = new TimerCanceller(200);
    private TimerCanceller onCanceller = new TimerCanceller(700);
    private boolean isAuto = false;

    public Collector(HardwareMap map) {
        collector = new CachingMotor(map.get(DcMotorEx.class, "collect"));
        tilt = new CachingServo(map.get(ServoImplEx.class, "collectorTilt"));
        gate = new CachingServo(map.get(ServoImplEx.class, "collectorGate"));

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tilt.setPwmRange(new PwmControl.PwmRange(1900,2400));
        gate.setPwmRange(new PwmControl.PwmRange(1240,1900));

        collector.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void reset() {
//        setGoal(Goal.RETRACT);
    }

    @Override
    public void update() {
        switch(goal) {
            case DEFAULT:
                break;
            case DEPLOY:
                deployCanceller.reset();
                setGoal(Goal.DEPLOYACTION);
                break;
            case DEPLOYACTION:
                deploy();
                if (deployCanceller.isConditionMet()) {
                    if (gateOverride) {
                        open();
                    } else {
                        close();
                    }
                    on();
                }
                break;
            case RETRACT:
                retractCanceller.reset();
                setGoal(Goal.RETRACTACTION);
                break;
            case RETRACTACTION:
                retract();
                if (retractCanceller.isConditionMet()) {
                    if (isAuto) {
                        off();
                        gateCanceller.reset();
                        setGoal(Goal.OPEN);
                    } else {
                        open();
                        offCanceller.reset();
                        setGoal(Goal.OFF);
                    }
                }
                break;
            case OFF:
                if (offCanceller.isConditionMet()) {
                    off();
                }
                break;
            case OPEN:
                if (gateCanceller.isConditionMet()) {
                    on();
                    open();
                    onCanceller.reset();
                    setGoal(Goal.OFFAUTO);
                }
                break;
            case OFFAUTO:
                if (onCanceller.isConditionMet()) {
                    off();
                }
                break;
        }
    }
    /*

     */

    @Override
    public String test() {
        return null;
    }

    public void on() {
        collector.setPower(Math.signum(sign) * COLLECT_POWER);
    }

    public void off() {
        collector.setPower(0);
    }

    public void deploy() {
        tilt.setPosition(1);
    }

    public void retract() {
        tilt.setPosition(0);
    }

    public void close() {
        gate.setPosition(1);
    }

    public void open() {
        gate.setPosition(0);
    }

    public void setSign(int sign) {
        this.sign = sign;
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    public double getGatePosition() {
        return gate.getPosition();
    }

    public void setGateOverride(boolean override) {
        gateOverride = override;
    }

    public double getTiltPosition() {
        return tilt.getPosition();
    }

    public void setAuto(boolean auto) {
        isAuto = auto;
    }
}