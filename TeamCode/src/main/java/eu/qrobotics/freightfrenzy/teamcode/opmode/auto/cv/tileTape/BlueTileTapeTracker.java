package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BlueTileTapeTracker extends TileTapeTracker {
    private double[] hslThresholdHue = {80.0, 100.0};
    private double[] hslThresholdSaturation = {150.0, 255.0};
    private double[] hslThresholdLuminance = {0.0, 255.0};
    private Mat mask, hsl, result;
    private Scalar count = new Scalar(0);

    public BlueTileTapeTracker(Point topLeft, Point bottomRight) {
        super(topLeft, bottomRight);
    }

    public double getCount() {
        return count.val[0];
    }

    @Override
    public Mat processFrame(Mat input) {
        if (mask == null) {
            mask = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            hsl = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            result = new Mat(input.height(), input.width(), CvType.CV_8UC3);
        }

        mask.setTo(new Scalar(0, 0, 0));
        result.setTo(new Scalar(0, 0, 0));
        Imgproc.rectangle(mask, topLeft, bottomRight, new Scalar(255, 255, 255), Core.FILLED);

        hslThreshold(input, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hsl);

        Core.bitwise_and(mask, mask, result, hsl);
        count = Core.sumElems(result);

        Imgproc.rectangle(hsl,
                topLeft,
                bottomRight,
                new Scalar(255, 0, 0), 4);

        return hsl;
    }
}