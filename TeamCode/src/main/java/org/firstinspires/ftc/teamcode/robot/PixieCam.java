package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.autonomous.BarcodePattern;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class PixieCam implements Component {



    public double team_element_x;
    public double team_element_y;

    public final double LEFT_X = 135;
    public final double MIDDLE_X = 197;
    public final double RIGHT_X = 240;


    I2cDeviceSynch pixyCam;
    BarcodePattern pos;

    public PixieCam(HardwareMap map)
    {
        pixyCam = map.i2cDeviceSynch.get("Pixy Cam");


        pixyCam.engage();


    }

    @Override
    public void reset() {
        pixyCam.disengage();
        pixyCam.engage();
    }

    @Override
    public void update() {
        team_element_x = 0xff&pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&pixyCam.read(0x51,5)[2];

    }

    @Override
    public String test() {
        return null;
    }

    public BarcodePattern return_barcode_pos()
    {
        if(team_element_x <= (LEFT_X+MIDDLE_X)/2.0)
        {
            pos = BarcodePattern.LEVELONE;
        }
        else if(team_element_x <= (MIDDLE_X+RIGHT_X)/2.0)
        {
            pos = BarcodePattern.LEVELTWO;
        }
        else
        {
            pos = BarcodePattern.LEVELTHREE;
        }
        return pos;
    }


}
