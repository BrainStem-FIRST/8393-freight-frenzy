package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVPipeline extends OpenCvPipeline {

    private static final int STREAM_W = 0;
    private static final int STREAM_L = 0;
    private static final int RECT1_W = 0;
    private static final int RECT1_L = 0;
    private static final int top_left1 = 0;
    private static final int top_right1 = 0;


    private static Point point_top_left1 = new Point(top_left1, top_right1);
    private static Point point_back_right1 = new Point(top_left1+RECT1_W, top_right1+RECT1_L);


    Mat Y = new Mat();
    Mat YConverted = new Mat();
    Mat Rect1_Y = new Mat();

    int avg;
    int avg_rect1;

    public void inputToY(Mat input)
    {
        Imgproc.cvtColor(input, YConverted, Imgproc.COLOR_RGB2YCrCb);
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }
}
