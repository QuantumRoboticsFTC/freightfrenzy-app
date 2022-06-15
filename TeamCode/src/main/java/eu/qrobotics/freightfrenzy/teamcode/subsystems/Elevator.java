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

    private static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double THRESHOLD_DOWN = 1;
    public static double THRESHOLD_DOWN_LEVEL_1 = 4;
    public static double THRESHOLD_DOWN_LEVEL_2 = 8;
    public static double THRESHOLD_DOWN_LEVEL_3 = 10;
    public static double THRESHOLD = 0.5;
    public static double THRESHOLD_LEVEL_1 = 1;
    public static double THRESHOLD_LEVEL_2 = 3;
    public static double THRESHOLD_LEVEL_3 = 5;
//    public static double DOWN_POWER_1 = -0.01;
//    public static double DOWN_POWER_2 = -0.02;
//    public static double DOWN_POWER_3 = -0.05;
//    public static double DOWN_POWER_4 = -0.1;
    public static double DOWN_POWER_1 = 0;
    public static double DOWN_POWER_2 = 0;
    public static double DOWN_POWER_3 = 0;
    public static double DOWN_POWER_4 = 0;
    public static double HOLD_POWER = 0.15;
    public static double LEVEL_1_POWER = 0.4;
    public static double LEVEL_2_POWER = 0.6;
    public static double LEVEL_3_POWER = 0.85;
    public static double LEVEL_4_POWER = 1;

    public enum ElevatorMode {
        DISABLED,
        DOWN,
        UP,
        MANUAL
    }

    public enum TargetHeight {
        LOW(0) {
            @Override
            public TargetHeight previous() {
                return this;
            }
        },
        MID(5.5),
        HIGH(12),
        AUTO_HIGH(14.3),
        CAPSTONE(12) {
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
    private NanoClock clock = NanoClock.system();

    public ElevatorMode elevatorMode;
    public TargetHeight targetHeight;

//    public DcMotorEx motorLeft;
    public DcMotorEx motorRight;
    private Robot robot;

    Elevator(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;

//        motorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

//        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

//        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        downPosition = getEncoder();

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
        return getDistanceLeft() > THRESHOLD;
    }

    public double getDistanceLeft() {
        return Math.abs(getError());
    }

    public double getError() {
        if(elevatorMode == ElevatorMode.DOWN)
            return -getCurrentHeight();
        return (targetHeight.getHeight() + offsetPosition) - getCurrentHeight();
    }

    public void reset() {
        downPosition = getEncoder();
    }

    private int singleBrake = 1000;

    @Override
    public void update() {
        switch (elevatorMode) {
            case DISABLED:
                setPower(0);
            case DOWN:
                offsetPosition = 0;
                if (getCurrentHeight() <= THRESHOLD_DOWN)
                    setPower(0);
//                else if(getCurrentHeight() <= THRESHOLD_DOWN_LEVEL_1)
//                    setPower(DOWN_POWER_1);
//                else if(getCurrentHeight() <= THRESHOLD_DOWN_LEVEL_2)
//                    setPower(DOWN_POWER_2);
//                else if(getCurrentHeight() <= THRESHOLD_DOWN_LEVEL_3)
//                    setPower(DOWN_POWER_3);
//                else
//                    setPower(DOWN_POWER_4);
                else if(getCurrentHeight() <= THRESHOLD_DOWN_LEVEL_1) {
//                    if(singleBrake < 3) {
//                        setPower(0.01);
//                        singleBrake++;
//                    }
//                    else {
                        setPower(-0.15);
//                    }
                }
                else {
                    singleBrake = 0;
                    setPower(0);
                }
                break;
            case UP:
                double error = getError();
                if (Math.abs(error) <= THRESHOLD)
                    setPower(HOLD_POWER);
                else if (Math.abs(error) <= THRESHOLD_LEVEL_1)
                    setPower(LEVEL_1_POWER * Math.signum(error));
                else if (Math.abs(error) <= THRESHOLD_LEVEL_2)
                    setPower(LEVEL_2_POWER * Math.signum(error));
                else if (Math.abs(error) <= THRESHOLD_LEVEL_3)
                    setPower(LEVEL_3_POWER * Math.signum(error));
                else
                    setPower(LEVEL_4_POWER * Math.signum(error));
                break;
            case MANUAL:
                setPower((getCurrentHeight() > 1 ? HOLD_POWER : 0) + manualPower);
                break;
        }
    }

    private double prevPower;

    private void setPower(double power) {
        if(power != prevPower) {
//            motorLeft.setPower(power);
            motorRight.setPower(power);
        }
        prevPower = power;
    }
}