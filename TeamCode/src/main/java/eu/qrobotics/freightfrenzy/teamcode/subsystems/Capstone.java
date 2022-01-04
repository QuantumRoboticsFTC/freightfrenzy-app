package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Capstone implements Subsystem {

    public enum CapstoneMode {
        DOWN,
        UP,
        OUTTAKE,
    }

    public static double CAPSTONE_DOWN_POSITION = 0.13;
    public static double CAPSTONE_UP_POSITION = 0.64;
    public static double CAPSTONE_OUTTAKE_POSITION = 0.88;

    public CapstoneMode capstoneMode;

    private Servo capstoneServo;

    Capstone(HardwareMap hardwareMap, Robot robot) {
        capstoneServo = hardwareMap.get(Servo.class, "capstoneServo");

        capstoneMode = CapstoneMode.UP;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch(capstoneMode) {
            case DOWN:
                capstoneServo.setPosition(CAPSTONE_DOWN_POSITION);
                break;
            case UP:
                capstoneServo.setPosition(CAPSTONE_UP_POSITION);
                break;
            case OUTTAKE:
                capstoneServo.setPosition(CAPSTONE_OUTTAKE_POSITION);
                break;
        }
    }
}
