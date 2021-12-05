package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.CachingMotor;

@Config
public class CarouselSpin implements Component {
    private DcMotorEx spin;

    public static double SPIN_POWER = 0.3; //+ for blue, - for red

    public CarouselSpin(HardwareMap map) {
        spin = map.get(DcMotorEx.class, "spin");

        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        spin.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0, 0));
    }

    @Override
    public void reset() {
        spin.setPower(0);
    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void runSpinSequence(double power) {
        spin.setPower(SPIN_POWER);
    }
}