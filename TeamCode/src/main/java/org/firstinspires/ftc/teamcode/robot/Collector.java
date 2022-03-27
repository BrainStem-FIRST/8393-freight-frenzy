package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;
import android.util.Log;

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
    private ServoImplEx tiltLeft;
    private ServoImplEx tiltRight;
    //private ServoImplEx gate;
    private ColorSensor colorSensor;

    private static final double COLLECT_POWER = 1;
    private static final float COLOR_THRESHOLD = 500;
    private static final double SCALE_FACTOR = 255;
    private static final int CURRENT_THRESHOLD = 1600;

    private RollingAverage currentRollingAverage = new RollingAverage(5);
    private int sign = 1;
    private Goal goal = Goal.DEFAULT;
    //private boolean gateOverride = false;
    private TimerCanceller deployCanceller = new TimerCanceller(75);
    private TimerCanceller retractCanceller = new TimerCanceller(200);
    private TimerCanceller offCanceller = new TimerCanceller(600);

    private boolean isAuto;
    private boolean fullRetract = true;

    public Collector(HardwareMap map, boolean isAuto) {
        this.isAuto = isAuto;
        collector = new CachingMotor(map.get(DcMotorEx.class, "collect"));
        tiltLeft = new CachingServo(map.get(ServoImplEx.class, "collectorTiltL"));
        tiltRight = new CachingServo(map.get(ServoImplEx.class, "collectorTiltR"));
        //gate = new CachingServo(map.get(ServoImplEx.class, "collectorGate"));
        colorSensor = map.colorSensor.get("color");

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //collector.setDirection(DcMotorSimple.Direction.REVERSE);

        //TO DO - update PWMs for collector  and right servos
        //intemediate for left is 1466
        tiltLeft.setPwmRange(new PwmControl.PwmRange(600,1760));
        //gate.setPwmRange(new PwmControl.PwmRange(1240,1900));
    }

    @Override
    public void reset() {
    }

    @Override
    public void update() {
        currentRollingAverage.addNumber((int)collector.getCurrent(CurrentUnit.MILLIAMPS));
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
                    /*if (gateOverride) {
                        open();
                    } else {
                        close();
                    }*/
                    on();
                }
                break;
            case RETRACT:
                Log.d("BrainSTEM", "goal=RETRACT");
                retract();
                retractCanceller.reset();
                if (fullRetract) setGoal(Goal.RETRACTACTION);
                break;
            case RETRACTACTION:
                if(!fullRetract || retractCanceller.isConditionMet()) {
                    //open();
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
        tiltLeft.setPosition(0);
        tiltRight.setPosition(0);
    }

    public void retract() {
        //new deploy position is
        tiltLeft.setPosition(0.75);
        tiltRight.setPosition(0.75);
        setSign(-1);
    }

    public void tiltInit() {

        tiltLeft.setPosition(1);
        tiltRight.setPosition(1);
    }

    public double getTiltPosition() {
        return tiltLeft.getPosition();
    }

    //Gate
    /*public void close() {
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
    }*/

    //Goal
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    //Sensors
    public boolean isFreightCollectedCurrentDraw() {
        return getCurrentDrawAverage() > CURRENT_THRESHOLD;
    }

    public int getCurrentDrawAverage() {
        return currentRollingAverage.getAverage();
    }

    public boolean isFreightCollectedColor() {
        return getBrightness() > COLOR_THRESHOLD;
    }

    public double getBrightness() {
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsv);
        return hsv[2];
    }

    public boolean isFreightCollected() {
        return isFreightCollectedColor() || isFreightCollectedCurrentDraw();
    }

    public void setRetractFull(boolean f) {
        fullRetract = f;
    }

    public boolean getRetractFull() {
        return fullRetract;
    }
}