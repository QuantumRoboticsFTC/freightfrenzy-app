package eu.qrobotics.freightfrenzy.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous
public class AutoRedWarehouse extends LinearOpMode {

    public static int LEFT_TSE_UP_X = 70;
    public static int LEFT_TSE_UP_Y = 40;
    public static int LEFT_TSE_DOWN_X = 250;
    public static int LEFT_TSE_DOWN_Y = 150;
    public static int CENTER_TSE_UP_X = 200;
    public static int CENTER_TSE_UP_Y = 40;
    public static int CENTER_TSE_DOWN_X = 420;
    public static int CENTER_TSE_DOWN_Y = 150;
    public static int RIGHT_TSE_UP_X = 420;
    public static int RIGHT_TSE_UP_Y = 40;
    public static int RIGHT_TSE_DOWN_X = 600;
    public static int RIGHT_TSE_DOWN_Y = 150;

    public static double ELEVATOR_THRESHOLD = 0.5;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true, Alliance.RED);
        robot.drivetrain.setPoseEstimate(TrajectoriesRedWarehouse.START_POSE);

//        TeamShippingElementTracker leftTSE = new TeamShippingElementTracker(
//                new Point(LEFT_TSE_UP_X, LEFT_TSE_UP_Y),
//                new Point(LEFT_TSE_DOWN_X, LEFT_TSE_DOWN_Y)
//        );
//        TeamShippingElementTracker centerTSE = new TeamShippingElementTracker(
//                new Point(CENTER_TSE_UP_X, CENTER_TSE_UP_Y),
//                new Point(CENTER_TSE_DOWN_X, CENTER_TSE_DOWN_Y)
//        );
//        TeamShippingElementTracker rightTSE = new TeamShippingElementTracker(
//                new Point(RIGHT_TSE_UP_X, RIGHT_TSE_UP_Y),
//                new Point(RIGHT_TSE_DOWN_X, RIGHT_TSE_DOWN_Y)
//        );
//
//        OpenCvTrackerApiPipeline trackerApiPipeline = new OpenCvTrackerApiPipeline();
//        trackerApiPipeline.addTracker(leftTSE);
//        trackerApiPipeline.addTracker(centerTSE);
//        trackerApiPipeline.addTracker(rightTSE);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
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

        List<Trajectory> trajectoriesA = TrajectoriesRedWarehouse.getTrajectoriesA();
        List<Trajectory> trajectoriesB = TrajectoriesRedWarehouse.getTrajectoriesB();
        List<Trajectory> trajectoriesC = TrajectoriesRedWarehouse.getTrajectoriesC();

        TSEPattern tsePattern = TSEPattern.LEFT;
        while (!opModeIsActive() && !isStopRequested()) {
//            double[] counts = {average(leftTSE.getCount()),
//                    average(centerTSE.getCount()),
//                    average(rightTSE.getCount())};
//            int maxIdx = 0;
//            double max = 0;
//            for (int i = 0; i < counts.length; i++) {
//                if (counts[i] > max) {
//                    max = counts[i];
//                    maxIdx = i;
//                }
//            }
//            if (maxIdx == 0) {
//                tsePattern = TSEPattern.LEFT;
//            } else if (maxIdx == 1) {
//                tsePattern = TSEPattern.MIDDLE;
//            } else {
//                tsePattern = TSEPattern.RIGHT;
//            }
//            telemetry.addData("TSE Pattern", tsePattern);
//            telemetry.update();
        }

//        webcam.stopStreaming();

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

        robot.start();

        robot.drivetrain.followTrajectorySync(trajectories.get(0));

        robot.capstone.capstoneMode = Capstone.CapstoneMode.DOWN;
        robot.sleep(0.5);
        robot.capstone.capstoneMode = Capstone.CapstoneMode.UP;

        robot.drivetrain.followTrajectorySync(trajectories.get(1));

        switch(tsePattern) {
            case LEFT:
                robot.elevator.setTargetHeight(Elevator.TargetHeight.LOW);
                break;
            case MIDDLE:
                robot.elevator.setTargetHeight(Elevator.TargetHeight.MID);
                break;
            case RIGHT:
                robot.elevator.setTargetHeight(Elevator.TargetHeight.HIGH);
                break;
        }
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.UP);
        while(robot.elevator.getDistanceLeft() < ELEVATOR_THRESHOLD) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.sleep(0.2);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.DOWN);

        robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;

        robot.drivetrain.followTrajectorySync(trajectories.get(2));

        robot.intake.intakeMode = Intake.IntakeMode.IN;

        robot.distanceSensorLocalization.enabled = true;
        robot.sleep(0.2);
        robot.drivetrain.setPoseEstimate(robot.distanceSensorLocalization.getRobotPose());
        robot.distanceSensorLocalization.enabled = false;

        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.intake.intakeRotation = Intake.IntakeRotation.UP;

        robot.drivetrain.followTrajectorySync(trajectories.get(3));

        robot.elevator.setTargetHeight(Elevator.TargetHeight.HIGH);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.UP);
        while(robot.elevator.getDistanceLeft() < ELEVATOR_THRESHOLD) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.sleep(0.2);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.DOWN);

        robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;

        robot.drivetrain.followTrajectorySync(trajectories.get(4));

        robot.intake.intakeMode = Intake.IntakeMode.IN;

        robot.distanceSensorLocalization.enabled = true;
        robot.sleep(0.2);
        robot.drivetrain.setPoseEstimate(robot.distanceSensorLocalization.getRobotPose());
        robot.distanceSensorLocalization.enabled = false;

        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.intake.intakeRotation = Intake.IntakeRotation.UP;

        robot.drivetrain.followTrajectorySync(trajectories.get(5));

        robot.elevator.setTargetHeight(Elevator.TargetHeight.HIGH);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.UP);
        while(robot.elevator.getDistanceLeft() < ELEVATOR_THRESHOLD) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.sleep(0.2);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.DOWN);

        robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;

        robot.drivetrain.followTrajectorySync(trajectories.get(6));

        robot.intake.intakeMode = Intake.IntakeMode.IN;

        robot.distanceSensorLocalization.enabled = true;
        robot.sleep(0.2);
        robot.drivetrain.setPoseEstimate(robot.distanceSensorLocalization.getRobotPose());
        robot.distanceSensorLocalization.enabled = false;

        robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        robot.intake.intakeRotation = Intake.IntakeRotation.UP;

        robot.drivetrain.followTrajectorySync(trajectories.get(7));

        robot.elevator.setTargetHeight(Elevator.TargetHeight.HIGH);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.UP);
        while(robot.elevator.getDistanceLeft() < ELEVATOR_THRESHOLD) {
            robot.sleep(0.01);
        }
        robot.arm.armMode = Arm.ArmMode.BACK;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        robot.sleep(0.2);
        robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        robot.arm.armMode = Arm.ArmMode.FRONT;
        robot.sleep(0.2);
        robot.elevator.setElevatorMode(Elevator.ElevatorMode.DOWN);

        robot.drivetrain.followTrajectorySync(trajectories.get(8));

    }

    private double average(Scalar s) {
        if (s == null || s.val == null)
            return 0;
        return Arrays.stream(s.val).average().orElse(0);
    }
}
