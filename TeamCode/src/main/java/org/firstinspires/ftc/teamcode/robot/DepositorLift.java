package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class DepositorLift implements Component {
    private DcMotorEx lift;
    private ServoImplEx deposit;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;

    private static final double LIFT_UP_POWER = 0.85;
    private static final double LIFT_DOWN_POWER = 0.4;

    public DepositorLift(HardwareMap map, Telemetry telemetry) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        deposit = new CachingServo(map.get(ServoImplEx.class, "depositServo"));
        extend = new CachingServo(map.get(ServoImplEx.class, "extendServo"));
        shippingElementGrab = new CachingServo(map.get(ServoImplEx.class, "shippingElementGrab"));

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));

        deposit.setPwmRange(new PwmControl.PwmRange(0,0));
        extend.setPwmRange(new PwmControl.PwmRange(0,0));
        shippingElementGrab.setPwmRange(new PwmControl.PwmRange(0,0));
    }

    @Override
    public void reset() {
        liftDown();
        retractDeposit();
        retractExtension();
        retractSEGrab();
    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void liftUp() {
        lift.setPower(LIFT_UP_POWER);
    }

    public void liftDown() {
        lift.setPower(LIFT_DOWN_POWER);
    }

    public void deposit() {
        deposit.setPosition(1);
    }

    public void retractDeposit() {
        deposit.setPosition(0);
    }

    public void extend() {
        extend.setPosition(1);
    }

    public void retractExtension() {
        extend.setPosition(0);
    }

    public void deploySEGrab() {
        shippingElementGrab.setPosition(1);
    }

    public void retractSEGrab() {
        shippingElementGrab.setPosition(0);
    }
}