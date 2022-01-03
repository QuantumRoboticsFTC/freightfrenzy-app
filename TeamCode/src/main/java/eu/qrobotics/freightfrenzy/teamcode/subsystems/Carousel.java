package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel implements Subsystem {

    public double carouselPower;

    private DcMotor carouselMotor;

    Carousel(HardwareMap hardwareMap, Robot robot) {
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        carouselPower = 0;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        carouselMotor.setPower(carouselPower);
    }
}
