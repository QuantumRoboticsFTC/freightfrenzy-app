package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvTracker;

public class TeamShippingElementTracker extends OpenCvTracker {
    private Point pt1, pt2;
    private double[] hslThresholdHue = {100.0, 140.0};
    private double[] hslThresholdSaturation = {55.0, 255.0};
    private double[] hslThresholdLuminance = {0.0, 255.0};
    private Mat mask, hsl, result;
    private Scalar count;

    public TeamShippingElementTracker(Point leftUp, Point rightDown) {
        pt1 = leftUp;
        pt2 = rightDown;
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
        Imgproc.rectangle(mask, pt1, pt2, new Scalar(255, 255, 255), Core.FILLED);

        hslThreshold(input, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hsl);

        Core.bitwise_and(mask, mask, result, hsl);
        count = Core.sumElems(result);


        Imgproc.rectangle(hsl,
                pt1,
                pt2,
                new Scalar(255, 0, 0), 4);

        return hsl;
    }

    public Scalar getCount() {
        return count;
    }

    /**
     * Segment an image based on hue, saturation, and luminance ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue   The min and max hue
     * @param sat   The min and max saturation
     * @param lum   The min and max luminance
     * @param out   The image in which to store the output.
     */
    private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HLS);
        Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                new Scalar(hue[1], lum[1], sat[1]), out);
    }
}