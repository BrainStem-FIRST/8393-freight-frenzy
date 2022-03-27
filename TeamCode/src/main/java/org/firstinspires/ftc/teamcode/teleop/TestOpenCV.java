package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.OpenCVPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class TestOpenCV extends OpMode {

    public OpenCvWebcam webcam = null;

    @Override
    public void init() {
        WebcamName name = hardwareMap.get(WebcamName.class, "OakD");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name, webcamID);
        OpenCVPipeline pipeline = new OpenCVPipeline();
        webcam.setPipeline(pipeline);
    }

    @Override
    public void loop() {

    }
}
