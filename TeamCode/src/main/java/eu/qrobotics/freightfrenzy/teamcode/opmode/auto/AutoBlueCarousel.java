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
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.BlueTileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.RedTileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.cv.tileTape.TileTapeTracker;
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories.TrajectoriesBlueCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.CapstoneArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.HorizontalArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.IntakeCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
@Autonomous
public class AutoBlueCarousel extends LinearOpMode {

    private OpenCvCamera webcam;
    private TileTapeTracker leftTracker;
    private TileTapeTracker rightTracker;

    public static Alliance alliance = Alliance.BLUE;

    public static int LEFT_TOP_Y = 1150;
    public static int LEFT_BOTTOM_Y = 1300;
    public static int LEFT_LEFT_X = 0;
    public static int LEFT_RIGHT_X = 300;
    public static int RIGHT_TOP_Y = 1150;
    public static int RIGHT_BOTTOM_Y = 1300;
    public static int RIGHT_LEFT_X = 500;
    public static int RIGHT_RIGHT_X = 780;

    public static double ELEVATOR_THRESHOLD = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, Alliance.BLUE);
        robot.drivetrain.setPoseEstimate(TrajectoriesBlueCarousel.START_POSE);

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

        List<Trajectory> trajectories = TrajectoriesBlueCarousel.getTrajectories();

        TSEPattern tsePattern = TSEPattern.RIGHT;
        while (!opModeIsActive() && !isStopRequested()) {
            double[] counts = new double[] {
                    leftTracker.getCount(),
                    rightTracker.getCount(),
                    100000
            };
            int minIdx = 0;
            double min = Double.POSITIVE_INFINITY;
            for (int i = 0; i < counts.length; i++) {
                if (counts[i] < min) {
                    min = counts[i];
                    minIdx = i;
                }
            }
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

        webcam.closeCameraDeviceAsync(() -> {});

//        tsePattern = TSEPattern.RIGHT;
        robot.start();

        robot.drivetrain.followTrajectorySync(trajectories.get(0));


        switch(tsePattern) {
            case LEFT:
                robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
                break;
            case MIDDLE:
                robot.elevator.targetHeight = Elevator.TargetHeight.MID;
                break;
            case RIGHT:
                robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HIGH;
                break;
        }
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        if (tsePattern == TSEPattern.LEFT) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.LOW;
        } else if (tsePattern == TSEPattern.RIGHT) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.AUTO_HIGH;
        } else {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.NORMAL;
        }
        robot.arm.armMode = Arm.ArmMode.UP;
        robot.sleep(0.8);
        while(robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        if (tsePattern == TSEPattern.RIGHT) {
            robot.arm.armMode = Arm.ArmMode.LOW;
        } else {
            robot.arm.armMode = Arm.ArmMode.HIGH;
        }
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.2);

        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN_SLOW;
        robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.CAROUSEL;
        robot.drivetrain.followTrajectorySync(trajectories.get(1));

        while(robot.elevator.getCurrentHeight() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.drivetrain.followTrajectory(trajectories.get(2));
//        robot.sleep(1);
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN_SLOW;
        robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.CAROUSEL;
        robot.sleep(5);
        robot.drivetrain.followTrajectorySync(trajectories.get(3));
        robot.sleep(0.3);
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
        robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
        robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
        robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;

        robot.drivetrain.followTrajectorySync(trajectories.get(4));
//        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.01);
//        }
//        robot.sleep(3);
        robot.drivetrain.followTrajectorySync(trajectories.get(5));
//        robot.sleep(3);
//        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.01);
//        }
        robot.drivetrain.followTrajectorySync(trajectories.get(6));
//        robot.sleep(3);
//        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.01);
//        }

        robot.drivetrain.followTrajectorySync(trajectories.get(7));
        robot.drivetrain.followTrajectorySync(trajectories.get(8));
        robot.drivetrain.followTrajectorySync(trajectories.get(9));
        robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
        robot.sleep(0.2);
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT;
        robot.sleep(1);
//        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.01);
//        }
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
        robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HIGH;
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.AUTO_HIGH;
        robot.arm.armMode = Arm.ArmMode.UP;
        robot.sleep(0.8);
        while(robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.HIGH;
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.4);


//        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
//        robot.sleep(0.3);
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
        robot.sleep(0.2);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.drivetrain.followTrajectorySync(trajectories.get(10));
        robot.drivetrain.followTrajectorySync(trajectories.get(11));
        robot.capstoneArm.armmode = CapstoneArm.ArmMode.AUTO_PARK;
        robot.sleep(2);
        while(robot.elevator.getCurrentHeight() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.stop();
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
