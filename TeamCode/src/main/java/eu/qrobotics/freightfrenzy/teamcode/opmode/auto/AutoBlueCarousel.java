package eu.qrobotics.freightfrenzy.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import java.util.Arrays;
import java.util.List;

import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.TSEPattern;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.TeamShippingElementTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories.TrajectoriesBlueCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.HorizontalArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.IntakeCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
@Autonomous
public class AutoBlueCarousel extends LinearOpMode {

    public static int LEFT_TSE_UP_X = 50;
    public static int LEFT_TSE_UP_Y = 1100;
    public static int LEFT_TSE_DOWN_X = 450;
    public static int LEFT_TSE_DOWN_Y = 1500;
    public static int CENTER_TSE_UP_X = 700;
    public static int CENTER_TSE_UP_Y = 1100;
    public static int CENTER_TSE_DOWN_X = 1000;
    public static int CENTER_TSE_DOWN_Y = 1500;
//    public static int RIGHT_TSE_UP_X = 1525;
//    public static int RIGHT_TSE_UP_Y = 685;
//    public static int RIGHT_TSE_DOWN_X = 1850;
//    public static int RIGHT_TSE_DOWN_Y = 1080;

    public static double ELEVATOR_THRESHOLD = 2;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);

        Robot robot = new Robot(this, true, Alliance.BLUE);
        robot.drivetrain.setPoseEstimate(TrajectoriesBlueCarousel.START_POSE);

        TeamShippingElementTracker leftTSE = new TeamShippingElementTracker(
                new Point(LEFT_TSE_UP_X, LEFT_TSE_UP_Y),
                new Point(LEFT_TSE_DOWN_X, LEFT_TSE_DOWN_Y)
        );
        TeamShippingElementTracker centerTSE = new TeamShippingElementTracker(
                new Point(CENTER_TSE_UP_X, CENTER_TSE_UP_Y),
                new Point(CENTER_TSE_DOWN_X, CENTER_TSE_DOWN_Y)
        );
//        TeamShippingElementTracker rightTSE = new TeamShippingElementTracker(
//                new Point(RIGHT_TSE_UP_X, RIGHT_TSE_UP_Y),
//                new Point(RIGHT_TSE_DOWN_X, RIGHT_TSE_DOWN_Y)
//        );

        OpenCvTrackerApiPipeline trackerApiPipeline = new OpenCvTrackerApiPipeline();
        trackerApiPipeline.addTracker(leftTSE);
        trackerApiPipeline.addTracker(centerTSE);
//        trackerApiPipeline.addTracker(rightTSE);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1920,1080, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//        webcam.setPipeline(trackerApiPipeline);
//        webcam.showFpsMeterOnViewport(true);
//
//        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        List<Trajectory> trajectories = TrajectoriesBlueCarousel.getTrajectories();

        TSEPattern tsePattern = TSEPattern.RIGHT;
        while (!opModeIsActive() && !isStopRequested()) {
            double[] counts = {
                    average(leftTSE.getCount()),
                    average(centerTSE.getCount()),
//                    average(rightTSE.getCount())
                    4e6
            };
            int maxIdx = 0;
            double max = 0;
            for (int i = 0; i < counts.length; i++) {
                if (counts[i] > max) {
                    max = counts[i];
                    maxIdx = i;
                }
            }
            if (maxIdx == 0) {
                tsePattern = TSEPattern.LEFT;
            } else if (maxIdx == 1) {
                tsePattern = TSEPattern.MIDDLE;
            } else {
                tsePattern = TSEPattern.RIGHT;
            }
            telemetry.addData("TSE Pattern", tsePattern);
            telemetry.addData("Value Left", counts[0]);
            telemetry.addData("Value Mid", counts[1]);
            telemetry.addData("Value Right", counts[2]);
            telemetry.update();
        }

//        webcam.closeCameraDeviceAsync(() -> {});

        robot.start();

        robot.drivetrain.followTrajectorySync(trajectories.get(0));

        robot.drivetrain.followTrajectorySync(trajectories.get(1));

        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        switch(tsePattern) {
            case LEFT:
                robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
                break;
            case MIDDLE:
                robot.elevator.targetHeight = Elevator.TargetHeight.MID;
                break;
            case RIGHT:
                robot.elevator.targetHeight = Elevator.TargetHeight.HIGH;
                break;
        }
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;

        if(tsePattern == TSEPattern.LEFT) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.LOW;
            robot.arm.armMode = Arm.ArmMode.LOW;
        }
        else if(tsePattern == TSEPattern.MIDDLE) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.AUTO_MID;
            robot.arm.armMode = Arm.ArmMode.HIGH;
        }
        else {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.NORMAL;
            robot.arm.armMode = Arm.ArmMode.HIGH;
        }

        robot.sleep(0.7);

        while(robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        robot.sleep(0.3);

        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;

        robot.sleep(0.7);

        robot.arm.armMode = Arm.ArmMode.UP;
        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;

        robot.sleep(0.5);

        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;

        while(robot.elevator.getCurrentHeight() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        robot.arm.armMode = Arm.ArmMode.FRONT;

        robot.drivetrain.followTrajectorySync(trajectories.get(2));

        robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
        robot.intakeCarousel.spin();

        while (robot.intakeCarousel.isCarouselBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        robot.drivetrain.followTrajectorySync(trajectories.get(3));

        robot.sleep(1.0);

        robot.stop();
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}