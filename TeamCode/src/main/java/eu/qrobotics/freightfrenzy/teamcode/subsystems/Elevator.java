package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Elevator implements Subsystem {
    private static final double TICKS_PER_REV = 384.5;

    public static double SPOOL_RADIUS = 0.74; // in
    public static double GEAR_RATIO = 1.0; // output/input

    // the operating range of the elevator is restricted to [0, MAX_HEIGHT]
    public static double MAX_HEIGHT = 20.0;

    public static PIDCoefficients PID = new PIDCoefficients(0.2, 0, 0);

    public static double MAX_VEL = 20; // in/s
    public static double MAX_ACCEL = 10; // in/s^2
    public static double MAX_JERK = 0; // in/s^3

    public static double kV = 0.055;
    public static double kA = 0;
    public static double kStatic = 0;

    private static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 435;
    }

    public enum ElevatorMode {
        DISABLED,
        DOWN,
        UP,
//        MANUAL
    }

    public enum TargetHeight {
        LOW(0) {
            @Override
            public TargetHeight previous() {
                return this;
            }
        },
        MID(8),
        HIGH(14),
        CAP(20) {
            @Override
            public TargetHeight next() {
                return this;
            }
        };

        private final double height;

        TargetHeight(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

        public TargetHeight previous() {
            return values()[ordinal() - 1];
        }

        public TargetHeight next() {
            return values()[ordinal() + 1];
        }
    }

    private int downPosition;
    public double offsetPosition;
    public double manualPower;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime;

    private ElevatorMode elevatorMode;
    private TargetHeight targetHeight;

    public DcMotorEx motorLeft, motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

        motorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getEncoder();

        controller = new PIDFController(PID, kV, kA, kStatic, (h, v) -> h * 0.000001);

        elevatorMode = ElevatorMode.DOWN;
        targetHeight = TargetHeight.LOW;
        manualPower = 0;
        offsetPosition = 0;
    }

    private int getEncoder() {
        return motorRight.getCurrentPosition();
    }

    private double getEncoderVelocity() {
        return motorRight.getVelocity();
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(getEncoder() - downPosition);
    }

    public double getCurrentVelocity() {
        return encoderTicksToInches(getEncoderVelocity());
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public boolean isAutonomous() {
        return elevatorMode != ElevatorMode.DISABLED; // && elevatorMode != ElevatorMode.MANUAL;
    }

    private void generateMotion() {
        double height = 0;
        switch(elevatorMode) {
            case DOWN:
                height = 0;
                break;
            case UP:
                height = this.targetHeight.getHeight();
                break;
        }
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(getCurrentHeight(), 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();
        offsetPosition = 0;
    }

    public void setElevatorMode(ElevatorMode elevatorMode) {
        if (this.elevatorMode == elevatorMode)
            return;
        this.elevatorMode = elevatorMode;
        if(isAutonomous())
            generateMotion();
    }

    public ElevatorMode getElevatorMode() {
        return elevatorMode;
    }

    public void setTargetHeight(TargetHeight targetHeight) {
        if (this.targetHeight == targetHeight)
            return;
        this.targetHeight = targetHeight;
        if(isAutonomous())
            generateMotion();
    }

    public TargetHeight getTargetHeight() {
        return targetHeight;
    }

    @Override
    public void update() {
        switch (elevatorMode) {
            case DISABLED:
                setPower(0);
                break;
//            case MANUAL:
            case UP:
                if(manualPower != 0) {
                    setPower(manualPower);
                    if (profile != null) {
                        offsetPosition = getCurrentHeight() - profile.end().getX();
                    }
                    break;
                }
            case DOWN:
                double currentHeight = getCurrentHeight();
                double currentVelocity = getCurrentVelocity();
                if (isBusy()) {
                    // following a profile
                    double time = clock.seconds() - profileStartTime;
                    MotionState state = profile.get(time);
                    controller.setTargetPosition(state.getX());
                    controller.setTargetVelocity(state.getV());
                } else {
                    // just hold the position
                    controller.setTargetPosition(profile != null ? profile.end().getX() + offsetPosition : 0);
                    controller.setTargetVelocity(0);
                }
                setPower(controller.update(currentHeight, currentVelocity));
                break;
        }
    }

    private void setPower(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
}