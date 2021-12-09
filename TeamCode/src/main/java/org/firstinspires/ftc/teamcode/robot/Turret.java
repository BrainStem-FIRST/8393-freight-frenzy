package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CachingMotor;

public class Turret implements Component {
    private DcMotorEx turret;
    private DigitalChannel limit;

    private static final double TURRET_DEGREE_RANGE = Math.toRadians(140);
    private static final int TURRET_ENCODER_RANGE = 1023;
    private static final double MIN_ANGLE = Math.toRadians(-65);
    private static final double MAX_ANGLE = Math.toRadians(65);
    private static final double TURRET_POWER = 0.3;

    public Turret (HardwareMap map) {
        turret = new CachingMotor(map.get(DcMotorEx.class, "turret"));

        limit = map.digitalChannel.get("limit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void resetTurret() {
        while(!limit.getState()) {
            turret.setPower(0.3);
        }
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoSpinTurret(double theta) { //theta is in degrees, following the unit circle (0 is facing right on the x-axis)
        theta = Angle.normDelta(theta);
        if (theta < MIN_ANGLE || theta > MAX_ANGLE) {
            turret.setPower(0);
            return;
        }

        //TODO: negate theta if needed
        double targetPosition = theta * TURRET_ENCODER_RANGE / TURRET_DEGREE_RANGE;


        turret.setTargetPosition((int) Math.round(targetPosition));
        turret.setPower(TURRET_POWER);
    }

    public void spinTurret(Direction direction) {
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER : TURRET_POWER);
    }

    public void stopTurret() {
        turret.setPower(0);
    }

    public int encoderPosition() {
        return turret.getCurrentPosition();
    }
}
