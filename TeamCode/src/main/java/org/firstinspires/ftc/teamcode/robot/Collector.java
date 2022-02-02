package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

public class Collector implements Component {
    public enum Goal {
        DEFAULT, DEPLOY, DEPLOYACTION, RETRACT, RETRACTACTION, OFF
    }
    private DcMotorEx collector;
    private ServoImplEx tilt;
    private ServoImplEx gate;
    private ColorSensor color;

    private static final double COLLECT_POWER = 1;
    private static final float COLOR_THRESHOLD = 110;
    private static final double SCALE_FACTOR = 255;
    private static final int CURRENT_THRESHOLD = 0; //TODO: find

    private RollingAverage currentRollingAverage = new RollingAverage(5);
    private int sign = 1;
    private Goal goal = Goal.DEFAULT;
    private boolean gateOverride = false;
    private TimerCanceller deployCanceller = new TimerCanceller(75);
    private TimerCanceller retractCanceller = new TimerCanceller(200);
    private TimerCanceller offCanceller = new TimerCanceller(700);

    private boolean isAuto = false;

    public Collector(HardwareMap map) {
        collector = new CachingMotor(map.get(DcMotorEx.class, "collect"));
        tilt = new CachingServo(map.get(ServoImplEx.class, "collectorTilt"));
        gate = new CachingServo(map.get(ServoImplEx.class, "collectorGate"));
        color = map.colorSensor.get("color");

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector.setDirection(DcMotorSimple.Direction.REVERSE);

        tilt.setPwmRange(new PwmControl.PwmRange(600,1130));

        gate.setPwmRange(new PwmControl.PwmRange(1240,1900));
    }

    @Override
    public void reset() {
    }

    @Override
    public void update() {
        currentRollingAverage.addNumber((int)collector.getCurrent(CurrentUnit.AMPS));
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
                if(retractCanceller.isConditionMet()) {
                    open();
                    offCanceller.reset();
                    setGoal(Goal.OFF);
                }
                break;
            case OFF:
                if (offCanceller.isConditionMet()) {
                    off();
                    setGoal(Goal.DEFAULT);
                }
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }

    //Motor
    public void on() {
        collector.setPower(Math.signum(sign) * COLLECT_POWER);
    }

    public void off() {
        collector.setPower(0);
    }

    public void setSign(int sign) {
        this.sign = sign;
    }

    //Tilt
    public void deploy() {
        tilt.setPosition(0);
    }

    public void retract() {
        tilt.setPosition(0.838);
    }

    public void tiltInit() {
        tilt.setPosition(1);
    }

    public double getTiltPosition() {
        return tilt.getPosition();
    }

    //Gate
    public void close() {
        gate.setPosition(1);
    }

    public void open() {
        gate.setPosition(0);
    }

    public double getGatePosition() {
        return gate.getPosition();
    }

    public void setGateOverride(boolean override) {
        gateOverride = override;
    }

    //Goal
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    //Boolean setters
    public void setAuto(boolean auto) {
        isAuto = auto;
    }

    public boolean isFreightCollectedCurrentDraw() {
        return currentRollingAverage.getAverage() > CURRENT_THRESHOLD;
    }

    public boolean isFreightCollectedColor() {
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsv);
        return hsv[2] > COLOR_THRESHOLD;
    }

    public double getBrightness() {
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsv);
        return hsv[2];
    }
}