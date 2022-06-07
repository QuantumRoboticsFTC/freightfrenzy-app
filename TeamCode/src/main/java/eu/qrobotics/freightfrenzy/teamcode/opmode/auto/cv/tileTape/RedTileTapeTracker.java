package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedTileTapeTracker extends TileTapeTracker {
    private double[] hslThresholdHue1 = {0.0, 30.0};
    private double[] hslThresholdHue2 = {150.0, 180.0};
    private double[] hslThresholdSaturation = {55.0, 255.0};
    private double[] hslThresholdLuminance = {0.0, 255.0};
    private Mat mask, hsl1, hsl2, hsl, result;
    private Scalar count = new Scalar(0);

    public RedTileTapeTracker(Point topLeft, Point bottomRight) {
        super(topLeft, bottomRight);
    }

    public double getCount() {
        return count.val[0];
    }

    @Override
    public Mat processFrame(Mat input) {
        if (mask == null) {
            mask = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            hsl1 = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            hsl2 = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            hsl = new Mat(input.height(), input.width(), CvType.CV_8UC3);
            result = new Mat(input.height(), input.width(), CvType.CV_8UC3);
        }

        mask.setTo(new Scalar(0, 0, 0));
        result.setTo(new Scalar(0, 0, 0));
        Imgproc.rectangle(mask, topLeft, bottomRight, new Scalar(255, 255, 255), Core.FILLED);

        hslThreshold(input, hslThresholdHue1, hslThresholdSaturation, hslThresholdLuminance, hsl1);
        hslThreshold(input, hslThresholdHue2, hslThresholdSaturation, hslThresholdLuminance, hsl2);

        Core.bitwise_or(hsl1, hsl2, hsl);

        Core.bitwise_and(mask, mask, result, hsl);
        count = Core.sumElems(result);

        Imgproc.rectangle(hsl,
                topLeft,
                bottomRight,
                new Scalar(255, 0, 0), 4);

        return hsl;
    }
}