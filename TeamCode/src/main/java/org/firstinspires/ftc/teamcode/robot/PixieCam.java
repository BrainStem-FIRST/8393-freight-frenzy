package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.spartronics4915.lib.T265Camera;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class PixieCam implements Component {

    public double block_x;
    public double block_y;


    public double team_element_x;
    public double team_element_y;


    I2cDeviceSynch pixyCam;

    public PixieCam(HardwareMap map)
    {
        pixyCam = map.i2cDeviceSynch.get("Pixy Cam");
    }

    @Override
    public void reset() {
    }

    public void start() {
        pixyCam.engage();
    }

    public void stop() {
        pixyCam.disengage();
    }

    @Override
    public void update() {
        block_x = 0xff&pixyCam.read(0x50,5)[1];
        block_y = 0xff&pixyCam.read(0x50,5)[2];

        team_element_x = 0xff&pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&pixyCam.read(0x51,5)[2];
    }

    @Override
    public String test() {
        return null;
    }


}