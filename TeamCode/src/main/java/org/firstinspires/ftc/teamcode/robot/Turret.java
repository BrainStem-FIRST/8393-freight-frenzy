package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

import static java.lang.Thread.sleep;

public class Turret implements Component {
    private DcMotorEx turret;
    private DigitalChannel limit;

    private static final double TURRET_DEGREE_CONVERSION = Math.toRadians(145);
    private static final int TURRET_ENCODER_CONVERSION = 1073;
    private static final double MIN_ANGLE = Math.toRadians(70);
    private static final double MAX_ANGLE = Math.toRadians(290);
    private static final double TURRET_POWER = 0.5;
    private DepositorLift dL;
    private Telemetry telemetry;

    public Turret (HardwareMap map, DepositorLift dL, Telemetry telemetry) {
        this.dL = dL;
        this.telemetry = telemetry;
        turret = new CachingMotor(map.get(DcMotorEx.class, "turret"));

        limit = map.digitalChannel.get("turretLimit");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15,0,0,0));
    }

    @Override
    public void reset() {
        stopTurret();

    }

    @Override
    public void update() {

    }

    @Override
    public String test() {
        return null;
    }

    public void resetTurret() throws InterruptedException {
        Log.d("Turret", "Resetting");
        dL.flipMid();
        sleep(400);
        while(!limit.getState()) {
            turret.setPower(0.5);
        }
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        autoSpinTurret(Math.toRadians(180));
        while(turret.isBusy()) {
//            telemetry.addData("Current", turret.getCurrentPosition());
//            telemetry.addData("Target", turret.getTargetPosition());
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.addLine("Out of loop");
        telemetry.update();
        dL.flipIn();
    }

    public void setTurretHold() {
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setTargetPositionTolerance(3);
    }

    public void setTurretHoldPower() {
        turret.setPower(0.1);
    }

    public void autoSpinTurret(double theta) { //theta is in degrees, following the unit circle (0 is facing right on the x-axis)
        if (theta < MIN_ANGLE || theta > MAX_ANGLE) {
            turret.setPower(0);
            return;
        }

        //TODO: negate theta if needed
        double targetPosition = -(theta - Math.toRadians(35)) * ((double) TURRET_ENCODER_CONVERSION) / TURRET_DEGREE_CONVERSION;

//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_POWER);
    }

    public void spinTurret(Direction direction) {
        turret.setPower(direction == Direction.LEFT ? -TURRET_POWER : TURRET_POWER);
    }

    public void stopTurret() {
        turret.setPower(0);
    }

    public int encoderPosition() {
        return 0;
        //return turret.getCurrentPosition();
    }

    public boolean limitState() {
        return limit.getState();
    }
}
