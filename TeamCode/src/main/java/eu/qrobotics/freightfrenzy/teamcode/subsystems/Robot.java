package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public LynxModule hub1;
    public LynxModule hub2;

    public Drivetrain drivetrain;
    public IntakeCarousel intakeCarousel;
    public Elevator elevator;
    public HorizontalArm horizontalArm;
    public Arm arm;
    public OdometryRetract odometryRetract;
    public CapstoneArm capstoneArm;
//    public DistanceSensorLocalization distanceSensorLocalization;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private ExecutorService subsystemUpdateExecutor;
    public FtcDashboard dashboard;
    public MovingStatistics top250, top100, top10;
    public Map<Subsystem, MovingStatistics> top100Subsystems = new HashMap<>();

    private boolean started;

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private Runnable subsystemUpdateRunnable = () -> {
        double startTime, temp;
        while (!Thread.currentThread().isInterrupted()) {
            try {
                startTime = getCurrentTime(); // Get start time of update
//                hub1.clearBulkCache();
//                hub2.clearBulkCache();
//                hub1.getBulkData();
//                hub2.getBulkData();
                for (Subsystem subsystem : subsystems) { // Update all subsystems
                    if (subsystem == null) continue;
                    try {
                        double t = getCurrentTime();
                        subsystem.update();
                        top100Subsystems.get(subsystem).add(getCurrentTime() - t);
                        subsystemsWithProblems.remove(subsystem);
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        if (!subsystemsWithProblems.contains(subsystem))
                            subsystemsWithProblems.add(subsystem);
                    }
                }
                temp = getCurrentTime() - startTime; // Calculate loop time
                top10.add(temp); // Add loop time to different statistics
                top100.add(temp);
                top250.add(temp);
            } catch (Throwable t) {
                Log.wtf(TAG, t); // If we get here, then something really weird happened.
            }
        }
    };

    public Robot(OpMode opMode, boolean isAutonomous, Alliance alliance) {
        // Initialize statistics
        top10 = new MovingStatistics(10);
        top100 = new MovingStatistics(100);
        top250 = new MovingStatistics(250);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        hub1 = opMode.hardwareMap.get(LynxModule.class, "Control Hub");
        hub2 = opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 10");

        hub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        hub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //region Initialize subsystems
        subsystems = new ArrayList<>();

        try {
            drivetrain = new Drivetrain(opMode.hardwareMap, this, isAutonomous);
            subsystems.add(drivetrain);
        } catch (Exception e) {
            Log.w(TAG, "skipping Drivetrain", e);
        }

        try {
            intakeCarousel = new IntakeCarousel(opMode.hardwareMap, isAutonomous, alliance);
            subsystems.add(intakeCarousel);
        } catch (Exception e) {
            Log.w(TAG, "skipping Intake", e);
        }

        try {
            elevator = new Elevator(opMode.hardwareMap, this);
            subsystems.add(elevator);
        } catch (Exception e) {
            Log.w(TAG, "skipping Elevator", e);
        }

        try {
            horizontalArm = new HorizontalArm(opMode.hardwareMap, this);
            subsystems.add(horizontalArm);
        } catch (Exception e) {
            Log.w(TAG, "skipping HorizontalArm", e);
        }

        try {
            arm = new Arm(opMode.hardwareMap, this);
            subsystems.add(arm);
        } catch (Exception e) {
            Log.w(TAG, "skipping Arm", e);
        }

        try {
            odometryRetract = new OdometryRetract(opMode.hardwareMap, isAutonomous);
            subsystems.add(odometryRetract);
        } catch (Exception e) {
            Log.w(TAG, "skipping OdometryRetract", e);
        }

        try {
            capstoneArm = new CapstoneArm(opMode.hardwareMap, this);
            subsystems.add(capstoneArm);
        }
        catch (Exception e) {
            Log.w(TAG, "skipping Capstone");
        }

//        try {
//            distanceSensorLocalization = new DistanceSensorLocalization(opMode.hardwareMap, this, alliance);
//            subsystems.add(distanceSensorLocalization);
//        }
//        catch (Exception e) {
//            Log.w(TAG, "skipping DistanceSensorLocalization");
//        }

        //endregion
        for (Subsystem subsystem : subsystems) {
            top100Subsystems.put(subsystem, new MovingStatistics(100));
        }

        // Initialize update thread
        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        subsystemsWithProblems = new ArrayList<>();
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    public void stop() {
        if (started && subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
            for (Subsystem subsystem : subsystems) {
                subsystem.stop();
            }
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    //region Automatically Stop Robot Loop
    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
    }
    //endregion

    //region Global Warnings
    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        for (Subsystem subsystem : subsystemsWithProblems) {
            warnings.add("Problem with " + subsystem.getClass().getSimpleName());
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public boolean shouldTriggerWarningSound() {
        return false;
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        subsystemsWithProblems.clear();
    }
    //endregion
}
