package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeCarousel implements Subsystem {

    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW,
        CAROUSEL
    }

    public enum IntakeRotation {
        UP,
        DOWN
    }

    public static double INTAKE_IN_SPEED = 1.0;
    public static double INTAKE_IN_SLOW_SPEED = 0.7;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -1.0;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public static double FRONT_INTAKE_UP_POSITION = 0.275;
    public static double FRONT_INTAKE_DOWN_POSITION = 0.775;

    public static double REAR_INTAKE_UP_POSITION = 0.775;
    public static double REAR_INTAKE_DOWN_POSITION = 0.275;

    public static double START_VEL = 0.2;
    public static double ACCELERATION_RATE = 0.175; // power increase / second
    public static double TIME = 3.0; // seconds

    public static double ACCELERATION_RATE_AUTONOMOUS = 0.4; // power increase / second
    public static double TIME_AUTONOMOUS = 3.0; // seconds

    public IntakeMode frontIntakeMode;
    public IntakeMode rearIntakeMode;

    public IntakeRotation frontIntakeRotation;
    public IntakeRotation rearIntakeRotation;

    private IntakeMode prevFrontIntakeMode;
    private IntakeMode prevRearIntakeMode;

    private IntakeRotation prevFrontIntakeRotation;
    private IntakeRotation prevRearIntakeRotation;

    public DcMotorEx frontIntakeMotor;
    public DcMotorEx rearIntakeMotor;

    public Servo frontIntakeServo;
    public Servo rearIntakeServo;

    private boolean isAutonomous;

    IntakeCarousel(HardwareMap hardwareMap, boolean isAutonomous) {
        this.isAutonomous = isAutonomous;

        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        rearIntakeMotor = hardwareMap.get(DcMotorEx.class, "rearIntakeMotor");
        frontIntakeServo = hardwareMap.get(Servo.class, "frontIntakeServo");
        rearIntakeServo = hardwareMap.get(Servo.class, "rearIntakeServo");

        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontIntakeMode = IntakeMode.IDLE;
        rearIntakeMode = IntakeMode.IDLE;

        frontIntakeRotation = IntakeRotation.UP;
        rearIntakeRotation = IntakeRotation.UP;
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
        carouselPower = Math.min(carouselPower, 1);
        return carouselPower;
    }

    @Override
    public void update() {
        if(IS_DISABLED) return;

//        if(frontIntakeMode != prevFrontIntakeMode) {
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
            }
//        }
        if(frontIntakeMode == IntakeMode.CAROUSEL) {
            frontIntakeMotor.setPower(getCarouselPower());
        }
        prevFrontIntakeMode = frontIntakeMode;

//        if(rearIntakeMode != prevRearIntakeMode) {
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
            }
//        }
        if(rearIntakeMode == IntakeMode.CAROUSEL) {
            rearIntakeMotor.setPower(getCarouselPower());
        }
        prevRearIntakeMode = rearIntakeMode;

//        if(frontIntakeRotation != prevFrontIntakeRotation) {
            switch (frontIntakeRotation) {
                case UP:
                    frontIntakeServo.setPosition(FRONT_INTAKE_UP_POSITION);
                    break;
                case DOWN:
                    frontIntakeServo.setPosition(FRONT_INTAKE_DOWN_POSITION);
                    break;
            }
//        }
        prevFrontIntakeRotation = frontIntakeRotation;

//        if(rearIntakeRotation != prevRearIntakeRotation) {
            switch (rearIntakeRotation) {
                case UP:
                    rearIntakeServo.setPosition(REAR_INTAKE_UP_POSITION);
                    break;
                case DOWN:
                    rearIntakeServo.setPosition(REAR_INTAKE_DOWN_POSITION);
                    break;
            }
//        }
        prevRearIntakeRotation = rearIntakeRotation;
    }
}
