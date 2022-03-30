package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.util.TimerCanceller;

@Config
public class CarouselSpin implements Component {
    private CRServo spinLeft;
    private CRServo spinRight;

    //- for blue, + for red
    private double spinPowerAuto = 0.1;
    private double spinPowerTeleOp = 0.25;

    private AllianceColor color;

    public CarouselSpin(HardwareMap map, AllianceColor color) {
        this.color = color;
        spinLeft = map.crservo.get("spinLeft");
        spinRight = map.crservo.get("spinRight");
    }

    @Override
    public void reset() {
        spinLeft.setPower(0);
    }

    @Override
    public void update() {
    }

    @Override
    public String test() {
        return null;
    }

    public void onTeleOp() {
        spinLeft.setPower(color == AllianceColor.BLUE ? -spinPowerTeleOp : spinPowerTeleOp);
        spinRight.setPower(color == AllianceColor.BLUE ? -spinPowerTeleOp : spinPowerTeleOp);
    }

    public void onAuto() {
        spinLeft.setPower(color == AllianceColor.BLUE ? -spinPowerAuto : spinPowerAuto);
        spinRight.setPower(color == AllianceColor.BLUE ? -spinPowerAuto : spinPowerAuto);
    }

    public void off() {
        spinLeft.setPower(0);
        spinRight.setPower(0);
    }

    public void setPower(double power) {
        spinLeft.setPower(power);
        spinRight.setPower(power);
    }

    public double getPower() {
        return spinLeft.getPower();
    }
}