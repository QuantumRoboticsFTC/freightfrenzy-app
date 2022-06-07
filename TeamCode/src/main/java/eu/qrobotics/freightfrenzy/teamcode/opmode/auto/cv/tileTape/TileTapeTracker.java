package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvTracker;

public abstract class TileTapeTracker extends OpenCvTracker {
    public Point topLeft, bottomRight;

    public abstract double getCount();

    public TileTapeTracker(Point topLeft, Point bottomRight) {
        this.topLeft = topLeft;
        this.bottomRight = bottomRight;
    }

    void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
                      Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HLS);
        Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                new Scalar(hue[1], lum[1], sat[1]), out);
    }
}
