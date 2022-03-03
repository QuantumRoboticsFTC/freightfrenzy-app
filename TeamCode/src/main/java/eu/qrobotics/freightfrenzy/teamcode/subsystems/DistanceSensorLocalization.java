package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.DistanceSensorFiltered;

@Config
public class DistanceSensorLocalization implements Subsystem {
    public static Pose2d LEFT_SENSOR_POSE = new Pose2d(-6.5, 5.00, Math.toRadians(90));
    public static Pose2d RIGHT_SENSOR_POSE = new Pose2d(-6.5, -5.00, Math.toRadians(-90));
    public static Pose2d FRONT_SENSOR_POSE = new Pose2d(7.00, 5.00, Math.toRadians(0));

    public static double LEFT_WALL_Y = 72;
    public static double RIGHT_WALL_Y = -72;
    public static double FRONT_WALL_X = 72;

//    public DistanceSensorFiltered leftSensor;
    public DistanceSensorFiltered rightSensor;
    public DistanceSensorFiltered frontSensor;

    private Robot robot;
    private Alliance alliance;

    DistanceSensorLocalization(HardwareMap hardwareMap, Robot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;

//        leftSensor = new DistanceSensorFiltered(hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor"));
        rightSensor = new DistanceSensorFiltered(hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor"));
        frontSensor = new DistanceSensorFiltered(hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor"));
    }

    private boolean enabled = false;

    private Pose2d lastPose = new Pose2d();

    public Pose2d getRobotPose() {
        if(!enabled) {
            return null;
        }
        return lastPose;
    }

    private double mmToInches(double mm) {
        return mm / 25.4;
    }

    public void enable() {
        this.enabled = true;
//        leftSensor.reset();
        rightSensor.reset();
        frontSensor.reset();
    }

    public void disable() {
        this.enabled = false;
    }

    @Override
    public void update() {
        if(enabled) {
//            double leftDistance = mmToInches(leftSensor.getDistance());
            double rightDistance = mmToInches(rightSensor.getDistance());
            double frontDistance = mmToInches(frontSensor.getDistance());

            double angle = robot.drivetrain.getExternalHeading();

            double frontSensorAngle = FRONT_SENSOR_POSE.getHeading() + angle;
            double sensorWallXDistance = Math.sin(frontSensorAngle + Math.PI / 2) * frontDistance;
            double worldFrontSensorX = FRONT_WALL_X - sensorWallXDistance;
            double worldRobotX = worldFrontSensorX - FRONT_SENSOR_POSE.vec().rotated(angle).getX();

            double worldRobotY = 0;

            switch (alliance) {
                case RED:
                    double rightSensorAngle = RIGHT_SENSOR_POSE.getHeading() + angle;
                    double rightSensorWallYDistance = Math.cos(rightSensorAngle + Math.PI / 2) * rightDistance;
                    double worldRightSensorY = RIGHT_WALL_Y + rightSensorWallYDistance;
                    worldRobotY = worldRightSensorY - RIGHT_SENSOR_POSE.vec().rotated(angle).getY();
                    break;
                case BLUE:
//                    double leftSensorAngle = LEFT_SENSOR_POSE.getHeading() + angle;
//                    double leftSensorWallYDistance = Math.cos(leftSensorAngle + Math.PI / 2) * leftDistance;
//                    double worldLeftSensorY = LEFT_WALL_Y + leftSensorWallYDistance;
//                    worldRobotY = worldLeftSensorY - LEFT_SENSOR_POSE.vec().rotated(angle).getY();
                    break;
            }

            lastPose = new Pose2d(worldRobotX, worldRobotY, angle);
        }
    }
}
