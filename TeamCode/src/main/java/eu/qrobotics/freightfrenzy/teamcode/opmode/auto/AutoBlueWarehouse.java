package eu.qrobotics.freightfrenzy.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories.TrajectoriesBlueWarehouse;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.HorizontalArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.IntakeCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
@Autonomous
public class AutoBlueWarehouse extends LinearOpMode {

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
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true, Alliance.BLUE);
        robot.drivetrain.setPoseEstimate(TrajectoriesBlueWarehouse.START_POSE);

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

        List<Trajectory> trajectories = TrajectoriesBlueWarehouse.getTrajectories();

        TSEPattern tsePattern = TSEPattern.RIGHT;
        while (!opModeIsActive() && !isStopRequested()) {
            double[] counts =  new double[]{
                    leftTracker.getCount(),
                    rightTracker.getCount(),
                    100000
            } ;
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

        ElapsedTime safetyStopTimer = new ElapsedTime();

        webcam.closeCameraDeviceAsync(() -> {});

        if (!isStopRequested()) {
            robot.start();
        }

        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        switch (tsePattern) {
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
        robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;

        robot.drivetrain.followTrajectory(trajectories.get(0));

        robot.sleep(0.1);

        robot.arm.armMode = Arm.ArmMode.UP;

        robot.sleep(0.3);

        if (tsePattern == TSEPattern.LEFT) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.LOW;
        } else if (tsePattern == TSEPattern.RIGHT) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.AUTO_HIGH;
        } else {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.NORMAL;
        }

        robot.sleep(0.2);

        if (tsePattern == TSEPattern.RIGHT) {
            robot.arm.armMode = Arm.ArmMode.HIGH;
        } else {
            robot.arm.armMode = Arm.ArmMode.LOW;
        }

        robot.sleep(0.25);

        while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        if (tsePattern == TSEPattern.LEFT) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        } else {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.LOW;
        }
        robot.sleep(0.25);

        boolean safetyStop = false;

        for (int cycle = 0; cycle < TrajectoriesBlueWarehouse.CYCLE_COUNT; cycle++) {
            robot.drivetrain.followTrajectory(trajectories.get(1 + cycle * 3));

            robot.arm.armMode = Arm.ArmMode.UP;

            robot.sleep(0.3);

            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;

            robot.sleep(0.1);

            robot.arm.armMode = Arm.ArmMode.FRONT;

            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;

            while (robot.elevator.getCurrentHeight() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.sleep(0.07);

            if (!robot.intakeCarousel.hasElementRear()) {
                robot.drivetrain.followTrajectory(trajectories.get(2 + cycle * 3));

                while (robot.drivetrain.isBusy() && !robot.intakeCarousel.hasElementRear() && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);
                }

                robot.drivetrain.cancelTrajectory();
            }

            if (30 - safetyStopTimer.seconds() < 3) {
                safetyStop = true;
                break;
            }

            robot.drivetrain.followTrajectory(trajectories.get(3 + cycle * 3));

            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;

            robot.sleep(0.15);

            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;

            robot.sleep(0.1);

            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT;

            while (!robot.arm.hasElement() && opModeIsActive() && !isStopRequested() && robot.drivetrain.isBusy()) {
                robot.sleep(0.01);
            }
//            robot.sleep(1.0);
            if (robot.arm.hasElement()) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
                robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HIGH;
                robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;

                robot.arm.armMode = Arm.ArmMode.UP;

//            robot.sleep(0.1);
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;

                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.AUTO_HIGH;

                robot.sleep(0.35);

                robot.arm.armMode = Arm.ArmMode.HIGH;

                while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
                    robot.sleep(0.01);

                    while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
                        robot.sleep(0.01);
                    }
                    robot.sleep(0.1);

                    robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;

                    robot.sleep(0.25);
                }
            }
        }

        if (!safetyStop) {
            robot.drivetrain.followTrajectory(trajectories.get(TrajectoriesBlueWarehouse.CYCLE_COUNT * 3 + 1));

            robot.arm.armMode = Arm.ArmMode.FRONT;
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;

            while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.sleep(0.1);
        }

        robot.stop();
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
