package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

@Config
public class CarouselSpin implements Component {
    private enum Goal {
        START, STARTACTION, STOP
    }
    private CRServo spin;

    private static final int COUNT_MAX = 10;

    //+ for blue, - for red
    private double spinPower = 1;

    private TimerCanceller rampupCanceller = new TimerCanceller(100);

    private boolean isOn = false;
    private boolean previous = false;
    private boolean firstOn = false;

    private AllianceColor color;
    private int counter = 0;
    private Goal goal;

    public CarouselSpin(HardwareMap map) {
        spin = map.crservo.get("spin");

//        spin.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void reset() {
        spin.setPower(0);
    }

    @Override
    public void update() {
        switch(goal) {
            case START:
                rampupCanceller.reset();
                if (counter >= COUNT_MAX) {
                    goal = Goal.STOP;
                } else {
                    goal = Goal.STARTACTION;
                }
                break;
            case STARTACTION:
                spin.setPower(color == AllianceColor.BLUE ? spinPower : -spinPower);
                if (rampupCanceller.isConditionMet()) {
                    spinPower += 0.1;
                    counter++;
                    goal = Goal.START;
                }
                break;
            case STOP:
                spin.setPower(0);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }

    public void on(AllianceColor color) {
        spin.setPower(color == AllianceColor.BLUE ? spinPower : -spinPower);
//        this.color = color;
//        rampupCanceller.reset();
//        goal = Goal.START;
    }

    public void off() {
        spin.setPower(0);
//        goal = Goal.STOP;
    }

    public void setPower(double power) {
        spin.setPower(power);
    }

    public double getPower() {
        return spin.getPower();
    }
}