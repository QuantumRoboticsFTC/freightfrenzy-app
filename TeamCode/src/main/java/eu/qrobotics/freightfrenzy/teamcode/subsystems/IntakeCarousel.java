package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
public class IntakeCarousel implements Subsystem {

    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW,
        CAROUSEL,
        BUTTERFLY
    }

    public enum IntakeRotation {
        UP,
        DOWN,
        CAROUSEL
    }

    public enum ButterflyRotation {
        UP,
        DOWN
    }

    public static double INTAKE_IN_SPEED = 1.0;
    public static double INTAKE_IN_SLOW_SPEED = 0.3;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -1.0;
    public static double INTAKE_OUT_SLOW_SPEED = -0.3;
    public static double INTAKE_BUTTERFLY_SPEED = 1.0;

    public static double FRONT_INTAKE_UP_POSITION = 0.755;
    public static double FRONT_INTAKE_DOWN_POSITION = 0.25;
    public static double FRONT_INTAKE_CAROUSEL_POSITION = 0.65;

    public static double REAR_INTAKE_UP_POSITION = 0.28;
    public static double REAR_INTAKE_DOWN_POSITION = 0.787;
    public static double REAR_INTAKE_CAROUSEL_POSITION = 0.32;

    public static double FRONT_BUTTERFLY_UP_POSITION = 0.36;
    public static double FRONT_BUTTERFLY_DOWN_POSITION = 0.88;

    public static double REAR_BUTTERFLY_UP_POSITION = 0.73;
    public static double REAR_BUTTERFLY_DOWN_POSITION = 0.25;

    public static double START_VEL = 0.23;
    public static double ACCELERATION_RATE = 0.15; // power increase / second
    public static double TIME = 3.0; // seconds

    public static double ACCELERATION_RATE_AUTONOMOUS = 0.125; // power increase / second
    public static double MAX_VEL_AUTONOMOUS = 0.4; // power
    public static double TIME_AUTONOMOUS = 5.0; // seconds

    public IntakeMode frontIntakeMode;
    public IntakeMode rearIntakeMode;

    public IntakeRotation frontIntakeRotation;
    public IntakeRotation rearIntakeRotation;

    public ButterflyRotation frontButterflyRotation;
    public ButterflyRotation rearButterflyRotation;

    private IntakeMode prevFrontIntakeMode;
    private IntakeMode prevRearIntakeMode;

    private IntakeRotation prevFrontIntakeRotation;
    private IntakeRotation prevRearIntakeRotation;

    private ButterflyRotation prevFrontButterflyRotation;
    private ButterflyRotation prevRearButterflyRotation;

    private DcMotorEx frontIntakeMotor;
    private DcMotorEx rearIntakeMotor;

    private Servo frontIntakeServo;
    private Servo rearIntakeServo;

    private Servo frontButterflyServo;
    private Servo rearButterflyServo;

    public RevColorSensorV3 frontSensor;
    public RevColorSensorV3 rearSensor;

    private boolean isAutonomous;
    private Alliance alliance;

    IntakeCarousel(HardwareMap hardwareMap, boolean isAutonomous, Alliance alliance) {
        this.isAutonomous = isAutonomous;
        this.alliance = alliance;

        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        rearIntakeMotor = hardwareMap.get(DcMotorEx.class, "rearIntakeMotor");

        frontIntakeServo = hardwareMap.get(Servo.class, "frontIntakeServo");
        rearIntakeServo = hardwareMap.get(Servo.class, "rearIntakeServo");

        frontButterflyServo = hardwareMap.get(Servo.class, "butterflyServoFront");
        rearButterflyServo = hardwareMap.get(Servo.class, "butterflyServoRear");

        // DSServos are inverted
        frontIntakeServo.setDirection(Servo.Direction.REVERSE);
        rearIntakeServo.setDirection(Servo.Direction.REVERSE);

        frontSensor = hardwareMap.get(RevColorSensorV3.class, "frontIntakeSensor");
        rearSensor = hardwareMap.get(RevColorSensorV3.class, "rearIntakeSensor");

        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntakeMode = IntakeMode.IDLE;
        rearIntakeMode = IntakeMode.IDLE;

        frontIntakeRotation = IntakeRotation.UP;
        rearIntakeRotation = IntakeRotation.UP;

        frontButterflyRotation = ButterflyRotation.UP;
        rearButterflyRotation = ButterflyRotation.UP;
    }

    public static boolean IS_DISABLED = false;

    ElapsedTime timer = new ElapsedTime(0);

    public void spin() {
        timer.reset();
    }

    public void stopSpin() {
        timer = new ElapsedTime(0);
    }

    public boolean isCarouselBusy() {
        return timer.seconds() < getTargetTime();
    }

    private double getTargetTime() {
        return isAutonomous ? TIME_AUTONOMOUS : TIME;
    }

    private double getAcceleration() {
        return isAutonomous ? ACCELERATION_RATE_AUTONOMOUS : ACCELERATION_RATE;
    }

    private double getCarouselPower() {
        double carouselPower = 0;
        if(timer.seconds() < getTargetTime()) {
            carouselPower = START_VEL + getAcceleration() * timer.seconds();
        }
        carouselPower = Math.min(carouselPower, isAutonomous ? MAX_VEL_AUTONOMOUS : 1);
        return (alliance == Alliance.BLUE ? -1 : 1) * carouselPower;
    }

    public boolean hasElementFront() {
        return frontSensor.getDistance(DistanceUnit.MM) < 29;
    }

    public boolean hasElementRear() {
        return rearSensor.getDistance(DistanceUnit.MM) < 29;
    }

    @Override
    public void update() {
        if (IS_DISABLED) return;

        if (frontIntakeMode != prevFrontIntakeMode) {
            switch (frontIntakeMode) {
                case IN:
                    frontIntakeMotor.setPower(INTAKE_IN_SPEED);
                    break;
                case IN_SLOW:
                    frontIntakeMotor.setPower(INTAKE_IN_SLOW_SPEED);
                    break;
                case IDLE:
                    frontIntakeMotor.setPower(INTAKE_IDLE_SPEED);
                    break;
                case OUT:
                    frontIntakeMotor.setPower(INTAKE_OUT_SPEED);
                    break;
                case OUT_SLOW:
                    frontIntakeMotor.setPower(INTAKE_OUT_SLOW_SPEED);
                    break;
                case BUTTERFLY:
                    frontIntakeMotor.setPower(INTAKE_BUTTERFLY_SPEED);
                    break;
            }
        }
        if (frontIntakeMode == IntakeMode.CAROUSEL) {
            frontIntakeMotor.setPower(getCarouselPower());
            frontIntakeRotation = IntakeRotation.CAROUSEL;
        }
        prevFrontIntakeMode = frontIntakeMode;

        if (rearIntakeMode != prevRearIntakeMode) {
            switch (rearIntakeMode) {
                case IN:
                    rearIntakeMotor.setPower(INTAKE_IN_SPEED);
                    break;
                case IN_SLOW:
                    rearIntakeMotor.setPower(INTAKE_IN_SLOW_SPEED);
                    break;
                case IDLE:
                    rearIntakeMotor.setPower(INTAKE_IDLE_SPEED);
                    break;
                case OUT:
                    rearIntakeMotor.setPower(INTAKE_OUT_SPEED);
                    break;
                case OUT_SLOW:
                    rearIntakeMotor.setPower(INTAKE_OUT_SLOW_SPEED);
                    break;
                case BUTTERFLY:
                    rearIntakeMotor.setPower(INTAKE_BUTTERFLY_SPEED);
                    break;
            }
        }
        if (rearIntakeMode == IntakeMode.CAROUSEL) {
            rearIntakeMotor.setPower(-getCarouselPower());
            rearIntakeRotation = IntakeRotation.CAROUSEL;
        }
        prevRearIntakeMode = rearIntakeMode;

        if (frontIntakeRotation != prevFrontIntakeRotation) {
            switch (frontIntakeRotation) {
                case UP:
                    frontIntakeServo.setPosition(FRONT_INTAKE_UP_POSITION);
                    break;
                case DOWN:
                    frontIntakeServo.setPosition(FRONT_INTAKE_DOWN_POSITION);
                    break;
                case CAROUSEL:
                    frontIntakeServo.setPosition(FRONT_INTAKE_CAROUSEL_POSITION);
                    break;
            }
        }
        prevFrontIntakeRotation = frontIntakeRotation;

        if (rearIntakeRotation != prevRearIntakeRotation) {
            switch (rearIntakeRotation) {
                case UP:
                    rearIntakeServo.setPosition(REAR_INTAKE_UP_POSITION);
                    break;
                case DOWN:
                    rearIntakeServo.setPosition(REAR_INTAKE_DOWN_POSITION);
                    break;
                case CAROUSEL:
                    rearIntakeServo.setPosition(REAR_INTAKE_CAROUSEL_POSITION);
                    break;
            }
        }
        prevRearIntakeRotation = rearIntakeRotation;

        if (frontButterflyRotation != prevFrontButterflyRotation) {
            switch (frontButterflyRotation) {
                case UP:
                    frontButterflyServo.setPosition(FRONT_BUTTERFLY_UP_POSITION);
                    break;
                case DOWN:
                    frontButterflyServo.setPosition(FRONT_BUTTERFLY_DOWN_POSITION);
                    break;
            }
        }
        prevFrontButterflyRotation = frontButterflyRotation;

        if (rearButterflyRotation != prevRearButterflyRotation) {
            switch (rearButterflyRotation) {
                case UP:
                    rearButterflyServo.setPosition(REAR_BUTTERFLY_UP_POSITION);
                    break;
                case DOWN:
                    rearButterflyServo.setPosition(REAR_BUTTERFLY_DOWN_POSITION);
                    break;
            }
        }
        prevRearButterflyRotation = rearButterflyRotation;
    }
}
