package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@Config
public class Carousel implements Subsystem {

    private DcMotor carouselMotor;
    private boolean isAutonomous;

    public static double START_VEL = 0.2;
    public static double ACCELERATION_RATE = 0.175; // power increase / second
    public static double TIME = 3.0; // seconds

    public static double ACCELERATION_RATE_AUTONOMOUS = 0.4; // power increase / second
    public static double TIME_AUTONOMOUS = 3.0; // seconds

    Carousel(HardwareMap hardwareMap, Robot robot, boolean isAutonomous, Alliance alliance) {
        this.isAutonomous = isAutonomous;

        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (alliance == Alliance.RED) {
            carouselMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public static boolean IS_DISABLED = false;

    ElapsedTime timer = new ElapsedTime(0);

    public void spin() {
        timer.reset();
    }

    public void stopSpin() {
        timer = new ElapsedTime(0);
    }

    public boolean isBusy() {
        return timer.seconds() < getTargetTime();
    }

    private double getTargetTime() {
        return isAutonomous ? TIME_AUTONOMOUS : TIME;
    }

    private double getAcceleration() {
        return isAutonomous ? ACCELERATION_RATE_AUTONOMOUS : ACCELERATION_RATE;
    }

    private double getPower() {
        double carouselPower = 0;
        if(timer.seconds() < getTargetTime()) {
            carouselPower = START_VEL + getAcceleration() * timer.seconds();
        }
        carouselPower = Math.min(carouselPower, 1);
        return carouselPower;
    }

    private double prevPower;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        double power = getPower();
        if(power != prevPower) {
            carouselMotor.setPower(power);
        }
        prevPower = power;
    }
}
