package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
public class DistanceSensorLocalization implements Subsystem {
    public static Pose2d LEFT_SENSOR_POSE = new Pose2d(2.5, 6.72, Math.toRadians(90));
    public static Pose2d RIGHT_SENSOR_POSE = new Pose2d(2.5, -6.72, Math.toRadians(-90));
    public static Pose2d FRONT_SENSOR_POSE = new Pose2d(6.47, 5.39, Math.toRadians(0));

    public static double LEFT_WALL_Y = 72;
    public static double RIGHT_WALL_Y = -72;
    public static double FRONT_WALL_X = 72;

    public Rev2mDistanceSensor leftSensor;
    public Rev2mDistanceSensor rightSensor;
    public Rev2mDistanceSensor frontSensor;

    private Robot robot;
    private Alliance alliance;

    DistanceSensorLocalization(HardwareMap hardwareMap, Robot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;

        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistanceSensor");
        frontSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
    }

    public boolean enabled = false;

    private MovingStatistics leftSensorAverage = new MovingStatistics(16);
    private MovingStatistics rightSensorAverage = new MovingStatistics(16);
    private MovingStatistics frontSensorAverage = new MovingStatistics(16);

    public Pose2d getRobotPose() {
        if(!enabled) {
            return null;
        }
        double leftDistance = leftSensorAverage.getMean();
        double rightDistance = rightSensorAverage.getMean();
        double frontDistance = frontSensorAverage.getMean();
        double angle = robot.drivetrain.getExternalHeading();

        double frontSensorAngle = FRONT_SENSOR_POSE.getHeading() + angle;
        double sensorWallXDistance = Math.sin(frontSensorAngle + Math.PI / 2) * frontDistance;
        double worldFrontSensorX = FRONT_WALL_X - sensorWallXDistance;
        double worldRobotX = worldFrontSensorX - FRONT_SENSOR_POSE.vec().rotated(FRONT_SENSOR_POSE.getHeading()).getX();

        double worldRobotY = 0;

        switch (alliance) {
            case RED:
                double rightSensorAngle = RIGHT_SENSOR_POSE.getHeading() + angle;
                double rightSensorWallYDistance = Math.cos(rightSensorAngle + Math.PI / 2) * rightDistance;
                double worldRightSensorY = RIGHT_WALL_Y + rightSensorWallYDistance;
                worldRobotY = worldRightSensorY - RIGHT_SENSOR_POSE.vec().rotated(RIGHT_SENSOR_POSE.getHeading()).getY();
                break;
            case BLUE:
                double leftSensorAngle = LEFT_SENSOR_POSE.getHeading() + angle;
                double leftSensorWallYDistance = Math.cos(leftSensorAngle + Math.PI / 2) * leftDistance;
                double worldLeftSensorY = LEFT_WALL_Y + leftSensorWallYDistance;
                worldRobotY = worldLeftSensorY - LEFT_SENSOR_POSE.vec().rotated(LEFT_SENSOR_POSE.getHeading()).getY();
                break;
        }

        return new Pose2d(worldRobotX, worldRobotY, angle);
    }

    private double mmToInches(double mm) {
        return mm / 25.4;
    }

    @Override
    public void update() {
        if(!enabled) {
            leftSensorAverage.clear();
            rightSensorAverage.clear();
            frontSensorAverage.clear();
        }
        else {
            double front = frontSensor.getDistance(DistanceUnit.MM);
            double left = leftSensor.getDistance(DistanceUnit.MM);
            double right = rightSensor.getDistance(DistanceUnit.MM);

            if(front > 65000) {
                frontSensor.initialize();
            }

            if(left > 65000) {
                frontSensor.initialize();
            }

            if(right > 65000) {
                frontSensor.initialize();
            }

            frontSensorAverage.add(mmToInches(front));
            switch(alliance) {
                case RED:
                    rightSensorAverage.add(mmToInches(right));
                    break;
                case BLUE:
                    leftSensorAverage.add(mmToInches(left));
                    break;
            }
        }
    }
}
