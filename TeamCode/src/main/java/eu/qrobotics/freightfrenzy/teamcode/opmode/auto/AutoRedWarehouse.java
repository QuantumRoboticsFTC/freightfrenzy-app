package eu.qrobotics.freightfrenzy.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories.TrajectoriesRedWarehouse;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Capstone;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Intake;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
@Autonomous
public class AutoRedWarehouse extends LinearOpMode {

    public static int LEFT_TSE_UP_X = 450;
    public static int LEFT_TSE_UP_Y = 650;
    public static int LEFT_TSE_DOWN_X = 750;
    public static int LEFT_TSE_DOWN_Y = 1000;
    public static int CENTER_TSE_UP_X = 1100;
    public static int CENTER_TSE_UP_Y = 650;
    public static int CENTER_TSE_DOWN_X = 1400;
    public static int CENTER_TSE_DOWN_Y = 1000;
    public static int RIGHT_TSE_UP_X = 1600;
    public static int RIGHT_TSE_UP_Y = 650;
    public static int RIGHT_TSE_DOWN_X = 1920;
    public static int RIGHT_TSE_DOWN_Y = 1000;

    public static double ELEVATOR_THRESHOLD = 2;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, Alliance.RED);
        robot.drivetrain.setPoseEstimate(TrajectoriesRedWarehouse.START_POSE);

        TeamShippingElementTracker leftTSE = new TeamShippingElementTracker(
                new Point(LEFT_TSE_UP_X, LEFT_TSE_UP_Y),
                new Point(LEFT_TSE_DOWN_X, LEFT_TSE_DOWN_Y)
        );
        TeamShippingElementTracker centerTSE = new TeamShippingElementTracker(
                new Point(CENTER_TSE_UP_X, CENTER_TSE_UP_Y),
                new Point(CENTER_TSE_DOWN_X, CENTER_TSE_DOWN_Y)
        );
        TeamShippingElementTracker rightTSE = new TeamShippingElementTracker(
                new Point(RIGHT_TSE_UP_X, RIGHT_TSE_UP_Y),
                new Point(RIGHT_TSE_DOWN_X, RIGHT_TSE_DOWN_Y)
        );

        OpenCvTrackerApiPipeline trackerApiPipeline = new OpenCvTrackerApiPipeline();
        trackerApiPipeline.addTracker(leftTSE);
        trackerApiPipeline.addTracker(centerTSE);
        trackerApiPipeline.addTracker(rightTSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        webcam.setPipeline(trackerApiPipeline);
        webcam.showFpsMeterOnViewport(true);

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        List<Trajectory> trajectoriesA = TrajectoriesRedWarehouse.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesRedWarehouse.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesRedWarehouse.getTrajectoriesC();

        TSEPattern tsePattern = TSEPattern.RIGHT;
        while (!opModeIsActive() && !isStopRequested()) {
            double[] counts = {average(leftTSE.getCount()),
                    average(centerTSE.getCount()),
                    average(rightTSE.getCount())};
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
            telemetry.update();
        }

        webcam.closeCameraDeviceAsync(() -> {});

        List<Trajectory> trajectories;
        if(tsePattern == TSEPattern.LEFT) {
            trajectories = trajectoriesA;
        }
        else if(tsePattern == TSEPattern.MIDDLE) {
            trajectories = trajectoriesB;
        }
        else {
            trajectories = trajectoriesC;
        }

        if(!isStopRequested()) {
            robot.start();
        }

        robot.drivetrain.followTrajectorySync(trajectories.get(0));

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
//        robot.capstone.capstoneMode = Capstone.CapstoneMode.UP_CLEARANCE;
        while(robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.sleep(0.6);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
        robot.sleep(0.2);

        for (int cycle = 0; cycle < 3; cycle++) {
            robot.drivetrain.followTrajectory(trajectories.get(1 + cycle * 2));

            robot.sleep(0.2);
            robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
            robot.sleep(0.1);
            robot.arm.armMode = Arm.ArmMode.FRONT;
            robot.sleep(0.2);
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;

            while(robot.elevator.getCurrentHeight() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.sleep(0.3);

//            robot.capstone.capstoneMode = Capstone.CapstoneMode.UP;
            robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.OUT;
            robot.sleep(0.2);
            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.BLOCK;

            while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.distanceSensorLocalization.enabled = true;

            robot.sleep(0.8);

            Pose2d pose = robot.distanceSensorLocalization.getRobotPose();
            if(pose.getY() > -48) {
                pose = new Pose2d(pose.getX(), -65, pose.getHeading());
            }
            if(pose.getX() < 0) {
                pose = new Pose2d(43, pose.getY(), pose.getHeading());
            }
            robot.drivetrain.setPoseEstimate(pose);
            robot.distanceSensorLocalization.enabled = false;

            robot.drivetrain.followTrajectory(trajectories.get(2 + cycle * 2));

            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            robot.sleep(0.1);

            robot.intake.intakeMode = Intake.IntakeMode.IN;

            robot.sleep(0.05);

            robot.intake.intakeRotation = Intake.IntakeRotation.UP;

            robot.sleep(0.1);

            robot.intake.intakeMode = Intake.IntakeMode.IDLE;

            robot.sleep(0.1);

            robot.intake.intakeRotation = Intake.IntakeRotation.TRANSFER;
            robot.arm.armMode = Arm.ArmMode.TRANSFER;
            robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
            robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.RAMP;

            robot.sleep(1.0);

            robot.intake.intakeMode = Intake.IntakeMode.IN;
            robot.sleep(0.05);
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            robot.sleep(0.5);
            robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.BLOCK;
            robot.elevator.targetHeight = Elevator.TargetHeight.AUTO_HIGH;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
//            robot.capstone.capstoneMode = Capstone.CapstoneMode.UP_CLEARANCE;
            while (robot.elevator.getDistanceLeft() > ELEVATOR_THRESHOLD && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            robot.arm.armMode = Arm.ArmMode.BACK;

            robot.sleep(0.6);

            while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
                robot.sleep(0.01);
            }

            if(tsePattern == TSEPattern.MIDDLE) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            }
            else {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
            }
            robot.sleep(0.4);
        }

        robot.drivetrain.followTrajectory(trajectories.get(7));

        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.sleep(0.2);
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.sleep(0.3);
        robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
        robot.sleep(0.5);
        robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;
        robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.BLOCK;
        robot.intake.intakeMode = Intake.IntakeMode.IN;

        while (robot.drivetrain.isBusy() && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.01);
        }

        robot.sleep(0.1);

        robot.stop();
    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
