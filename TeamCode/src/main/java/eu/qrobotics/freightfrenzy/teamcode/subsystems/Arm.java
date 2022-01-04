package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm implements Subsystem {

    public enum ArmMode {
        FRONT,
        MID,
        BACK,
        CAPSTONE
    }

    public enum TrapdoorMode {
        CLOSED,
        OPEN,
        OPEN_REVERSE,
    }

    public static double ARM_FRONT_LEFT_POSITION = 0.04;
    public static double ARM_FRONT_RIGHT_POSITION = 0.96;
    public static double ARM_MID_LEFT_POSITION = 0.5;
    public static double ARM_MID_RIGHT_POSITION = 0.5;
    public static double ARM_BACK_LEFT_POSITION = 0.95;
    public static double ARM_BACK_RIGHT_POSITION = 0.05;
    public static double ARM_CAPSTONE_LEFT_POSITION = 0.11;
    public static double ARM_CAPSTONE_RIGHT_POSITION = 0.89;

    public static double TRAPDOOR_CLOSED_POSITION = 0.5;
    public static double TRAPDOOR_OPEN_POSITION = 0.35;
    public static double TRAPDOOR_OPEN_REVERSE_POSITION = 0.65;

    public ArmMode armMode;
    public TrapdoorMode trapdoorMode;

    private Servo armServoLeft, armServoRight;
    private Servo trapdoorServo;

    Arm(HardwareMap hardwareMap, Robot robot) {
        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        trapdoorServo = hardwareMap.get(Servo.class, "trapdoorServo");

        armMode = ArmMode.FRONT;
        trapdoorMode = TrapdoorMode.CLOSED;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (armMode) {
            case FRONT:
                armServoLeft.setPosition(ARM_FRONT_LEFT_POSITION);
                armServoRight.setPosition(ARM_FRONT_RIGHT_POSITION);
                break;
            case MID:
                armServoLeft.setPosition(ARM_MID_LEFT_POSITION);
                armServoRight.setPosition(ARM_MID_RIGHT_POSITION);
                break;
            case BACK:
                armServoLeft.setPosition(ARM_BACK_LEFT_POSITION);
                armServoRight.setPosition(ARM_BACK_RIGHT_POSITION);
                break;
            case CAPSTONE:
                armServoLeft.setPosition(ARM_CAPSTONE_LEFT_POSITION);
                armServoRight.setPosition(ARM_CAPSTONE_RIGHT_POSITION);
                break;
        }
        switch (trapdoorMode) {
            case CLOSED:
                trapdoorServo.setPosition(TRAPDOOR_CLOSED_POSITION);
                break;
            case OPEN:
                trapdoorServo.setPosition(TRAPDOOR_OPEN_POSITION);
                break;
            case OPEN_REVERSE:
                trapdoorServo.setPosition(TRAPDOOR_OPEN_REVERSE_POSITION);
                break;
        }
    }
}
