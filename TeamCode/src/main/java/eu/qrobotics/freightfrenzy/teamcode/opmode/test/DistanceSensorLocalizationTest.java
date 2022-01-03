package eu.qrobotics.freightfrenzy.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.AxesSigns;
import eu.qrobotics.freightfrenzy.teamcode.util.BNO055IMUUtil;
import eu.qrobotics.freightfrenzy.teamcode.util.DashboardUtil;

@TeleOp(name = "Distance Sensor Localization Test", group = "Test")
public class DistanceSensorLocalizationTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false, Alliance.RED);
        robot.distanceSensorLocalization.enabled = true;
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        Pose2d robotPose = robot.distanceSensorLocalization.getRobotPose();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, robotPose);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}

