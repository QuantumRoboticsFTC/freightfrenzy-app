package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Carousel implements Subsystem {

    private DcMotor carouselMotor;

    public static double ACCELERATION_RATE = 0.7; // power increase / second
    public static double TIME = 2.0; // seconds

    Carousel(HardwareMap hardwareMap, Robot robot) {
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static boolean IS_DISABLED = false;

    ElapsedTime timer = new ElapsedTime(0);

    public void spin() {
        timer.reset();
    }

    public void stopSpin() {
        timer = new ElapsedTime(0);
    }

    @Override
    public void update() {
        if(IS_DISABLED) return;
        double carouselPower = 0;
        if(timer.seconds() < TIME) {
            carouselPower = ACCELERATION_RATE * timer.seconds();
        }
        carouselMotor.setPower(carouselPower);
    }
}
