package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CapstoneArm implements Subsystem{

    public enum ArmMode {
        COLLECT,
        SCORE,
        DOUBLE_SCORE,
        IDLE,
        AUTO_PARK
    }
    public enum ClawMode {
        OPEN,
        CLOSED
    }

    public static double COLLECT_ARM_POSITION = 0.87;
    public static double SCORE_ARM_POSITION = 0.50;
    public static double DOUBLE_SCORE_ARM_POSITION = 0.47;
    public static double IDLE_ARM_POSITION = 0.4;
    public static double AUTO_PARK_ARM_POSITION = 0.25;

    public static double CLOSED_CLAW_POSITION = 0.785;
    public static double OPEN_CLAW_POSITION = 0.33;

    public ArmMode armmode;
    public ClawMode clawMode;

    private Servo armServo;
    private Servo clawServo;

    CapstoneArm (HardwareMap hardwareMap, Robot robot) {
        armServo = hardwareMap.get(Servo.class, "capstoneArmServo");
        clawServo = hardwareMap.get(Servo.class, "capstoneClawServo");

        armmode = ArmMode.IDLE;
        clawMode = ClawMode.OPEN;
    }

    public static boolean IS_DISABLED = false;
    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (armmode) {
            case IDLE:
                armServo.setPosition(IDLE_ARM_POSITION);
                break;
            case COLLECT:
                armServo.setPosition(COLLECT_ARM_POSITION);
                break;
            case SCORE:
                armServo.setPosition(SCORE_ARM_POSITION);
                break;
            case DOUBLE_SCORE:
                armServo.setPosition(DOUBLE_SCORE_ARM_POSITION);
                break;
            case AUTO_PARK:
                armServo.setPosition(AUTO_PARK_ARM_POSITION);
                break;
        }
        switch (clawMode) {
            case OPEN:
                clawServo.setPosition(OPEN_CLAW_POSITION);
                break;
            case CLOSED:
                clawServo.setPosition(CLOSED_CLAW_POSITION);
                break;
        }
    }
}
