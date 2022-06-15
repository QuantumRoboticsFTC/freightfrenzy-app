package eu.qrobotics.freightfrenzy.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import java.util.Arrays;

import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.TSEPattern;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.BlueTileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.RedTileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.TileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
@TeleOp(group = "Test")
public class CVTest extends OpMode {

    private OpenCvCamera webcam;
    private TileTapeTracker leftTracker;
    private TileTapeTracker rightTracker;

    public static Alliance alliance = Alliance.BLUE;

    public static int LEFT_TOP_Y = 1150;
    public static int LEFT_BOTTOM_Y = 1300;
    public static int LEFT_LEFT_X = 300;
    public static int LEFT_RIGHT_X = 600;
    public static int RIGHT_TOP_Y = 1150;
    public static int RIGHT_BOTTOM_Y = 1300;
    public static int RIGHT_LEFT_X = 800;
    public static int RIGHT_RIGHT_X = 1080;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        OpenCvTrackerApiPipeline pipeline = new OpenCvTrackerApiPipeline();

        if(alliance == Alliance.RED) {
            leftTracker = new RedTileTapeTracker(new Point(LEFT_LEFT_X, LEFT_TOP_Y), new Point(LEFT_RIGHT_X, LEFT_BOTTOM_Y));
            rightTracker = new RedTileTapeTracker(new Point(RIGHT_LEFT_X, RIGHT_TOP_Y), new Point(RIGHT_RIGHT_X, RIGHT_BOTTOM_Y));
        }
        else {
            leftTracker = new BlueTileTapeTracker(new Point(LEFT_LEFT_X, LEFT_TOP_Y), new Point(LEFT_RIGHT_X, LEFT_BOTTOM_Y));
            rightTracker = new BlueTileTapeTracker(new Point(RIGHT_LEFT_X, RIGHT_TOP_Y), new Point(RIGHT_RIGHT_X, RIGHT_BOTTOM_Y));
        }

        pipeline.addTracker(leftTracker);
        pipeline.addTracker(rightTracker);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        webcam.setPipeline(pipeline);
        webcam.showFpsMeterOnViewport(true);

        FtcDashboard.getInstance().startCameraStream(webcam, 30);
    }

    @Override
    public void loop() {
        double[] counts = alliance == Alliance.RED ? new double[]{
            leftTracker.getCount(),
            rightTracker.getCount(),
            100000
        } : new double[] {
            100000,
            leftTracker.getCount(),
            rightTracker.getCount()
        };
        int minIdx = 0;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < counts.length; i++) {
            if (counts[i] < min) {
                min = counts[i];
                minIdx = i;
            }
        }
        TSEPattern tsePattern;
        if (minIdx == 0) {
            tsePattern = TSEPattern.LEFT;
        } else if (minIdx == 1) {
            tsePattern = TSEPattern.MIDDLE;
        } else {
            tsePattern = TSEPattern.RIGHT;
        }
        telemetry.addData("TSE Pattern", tsePattern);
        telemetry.addData("Values", Arrays.toString(counts));
        telemetry.update();
    }

    @Override
    public void stop() {
        webcam.closeCameraDeviceAsync(() -> {});
        super.stop();
    }
}
