package eu.qrobotics.freightfrenzy.teamcode.opmode.test;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.IntakeCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.StickyGamepad;

@TeleOp(group = "Test")
public class IntakeOuttakeTransferTest extends OpMode {
    Robot robot;
    StickyGamepad stickyGamepad;

    @Override
    public void init() {
        robot = new Robot(this, false, Alliance.RED);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        stickyGamepad = new StickyGamepad(gamepad1);
    }

    @Override
    public void start() {
        robot.start();
    }

    double previous = 0;
    double previousTimestamp = 0;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime intakeUpTimer = new ElapsedTime(0);

    boolean prevHasElement = false;

    List<Double> values = new ArrayList<>();

    @Override
    public void loop() {
        stickyGamepad.update();

        if (stickyGamepad.a) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
        }
        else if (stickyGamepad.b) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            intakeUpTimer.reset();
            timer.reset();
        }

        if(0.6 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 0.7) {
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT;
        }

        if(1.5 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 1.6) {
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
        }

        boolean hasElement = robot.arm.hasElement();

        if(hasElement && !prevHasElement) {
            previousTimestamp = getRuntime();
            previous = timer.milliseconds();
            values.add(previous);
        }

        prevHasElement = hasElement;

        telemetry.addData("Distance", robot.arm.sensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Has element", hasElement);
        telemetry.addData("Last element timestamp", previousTimestamp);
        telemetry.addData("Last element value", previous);
        telemetry.addData("Measurements", values.size());
        telemetry.update();
    }

    @Override
    public void stop() {
        try {
            FileWriter writer = new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/FIRST/test.txt");
            writer.write(Arrays.toString(values.toArray()));
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.stop();
    }
}
