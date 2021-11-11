package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class Collector implements Component {
    private DcMotorEx collector;
    private ServoImplEx deploy;
    private ServoImplEx gate;

    private static final double COLLECT_POWER = 1;
    private int sign = 1;

    public Collector(HardwareMap map) {
        collector = new CachingMotor(map.get(DcMotorEx.class, "collectMotor"));
        deploy = new CachingServo(map.get(ServoImplEx.class, "collectorDeploy"));
        gate = new CachingServo(map.get(ServoImplEx.class, "collectorGate"));

        collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        deploy.setPwmRange(new PwmControl.PwmRange(1560,2315));
        gate.setPwmRange(new PwmControl.PwmRange(0,0));

        collector.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void reset() {
//        close();
        off();
        retract();
    }

    @Override
    public void update() {

    }

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
        deploy.setPosition(0);
    }

    public void retract() {
        deploy.setPosition(1);
    }

    public void close() {
        gate.setPosition(1);
    }

    public void open() {
        gate.setPosition(0);
    }

    public void startCollection() {
//        close();
        deploy();
    }

    public void stopCollection() {
//        open();
        retract();
    }

    public void setSign(int sign) {
        this.sign = sign;
    }
}