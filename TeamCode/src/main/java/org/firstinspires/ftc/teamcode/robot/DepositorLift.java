package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
        UP, DOWN, STOP
    }

    public enum Location {
        NEAR, FAR
    }

    public enum Height {
        LOW, MIDDLE, HIGH
    }

    private DcMotorEx lift;
    private ServoImplEx depositNear;
    private ServoImplEx depositFar;
    private ServoImplEx extend;
    private ServoImplEx shippingElementGrab;
    private DigitalChannel magnetSensor;
    private RevTouchSensor touchSensor;

    private static final double LIFT_UP_POWER = 0.85;
    private static final double LIFT_DOWN_POWER = 0.4;
    private static final int LIFT_TIMEOUT = 3000;

    private TimerCanceller liftCanceller = new TimerCanceller(LIFT_TIMEOUT);

    private Location depositLocation = Location.NEAR;
    private Height depositHeight = Height.HIGH;
    private boolean currentMagnetState = false;
    private boolean previousMagnetState = false;
    private boolean passedMiddle = false;
    private Goal goal = Goal.STOP;

    public DepositorLift(HardwareMap map, Telemetry telemetry) {
        lift = new CachingMotor(map.get(DcMotorEx.class, "lift"));
        depositNear = new CachingServo(map.get(ServoImplEx.class, "depositNearServo"));
        depositFar = new CachingServo(map.get(ServoImplEx.class, "depositFarServo"));
        extend = new CachingServo(map.get(ServoImplEx.class, "extendServo"));
        shippingElementGrab = new CachingServo(map.get(ServoImplEx.class, "shippingElementGrab"));
        magnetSensor = map.digitalChannel.get("magnetSensor");
        touchSensor = map.get(RevTouchSensor.class, "touchSensor");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));

        depositNear.setPwmRange(new PwmControl.PwmRange(0,0));
        depositFar.setPwmRange(new PwmControl.PwmRange(0,0));
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

        if (goal == Goal.STOP || liftCanceller.isConditionMet())
            stopLift();
        else if (goal == Goal.UP)
        {
            if (depositHeight == Height.LOW) {
                goal = Goal.STOP;
            }

            liftUp();
            currentMagnetState = !magnetSensor.getState();
            if (currentMagnetState != previousMagnetState) {
                if (depositHeight == Height.MIDDLE) {
                    goal = Goal.STOP;
                } else if (depositHeight == Height.HIGH && passedMiddle)  {
                    goal = Goal.STOP;
                } else {
                    passedMiddle = true;
                    currentMagnetState = false;
                    previousMagnetState = false;
                }
            }
        }
        else if (goal == Goal.DOWN)
        {
            liftDown();
            if (touchSensor.isPressed())
            {
                goal = Goal.STOP;
                passedMiddle = false;
                currentMagnetState = false;
                previousMagnetState = false;
            }
        }
    }

    @Override
    public String test() {
        return null;
    }

    private void liftUp() {
        lift.setPower(LIFT_UP_POWER);
    }

    private void liftDown() {
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

    public void setDepositLocation(Location location) {
        depositLocation = location;
    }

    public void setHeight(Height height) {
        depositHeight = height;
    }

    public void deposit() {
        if (depositLocation == Location.NEAR) {
            depositNear();
        } else {
            depositFar();
        }
    }

    private void depositNear() {
        depositNear.setPosition(1);
    }

    private void depositFar() {
        depositFar.setPosition(1);
    }

    public void retractDeposit() {
        depositNear.setPosition(0);
        depositFar.setPosition(0);
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