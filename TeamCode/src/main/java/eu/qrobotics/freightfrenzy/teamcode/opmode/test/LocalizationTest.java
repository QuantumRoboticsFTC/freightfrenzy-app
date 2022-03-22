package eu.qrobotics.freightfrenzy.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.DashboardUtil;

@TeleOp(group = "Test")
public class LocalizationTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this, false, Alliance.RED);
        robot.odometryRetract.down = true;
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 0.5);

        Pose2d currentPose = robot.drivetrain.getPoseEstimate();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
