package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm implements Subsystem {

    public enum ArmMode {
        FRONT,
        UP,
        HIGH,
        LOW,
        CAPSTONE_PICKUP,
        CAPSTONE_UP,
        CAPSTONE_PLACE
    }

    public enum TrapdoorMode {
        CLOSED,
        LOW,
        OPEN,
    }

    public static double ARM_FRONT_LEFT_POSITION = 0.035;
    public static double ARM_FRONT_RIGHT_POSITION = 0.965;
    public static double ARM_UP_LEFT_POSITION = 0.29;
    public static double ARM_UP_RIGHT_POSITION = 0.7;
    public static double ARM_HIGH_LEFT_POSITION = 0.44;
    public static double ARM_HIGH_RIGHT_POSITION = 0.55;
    public static double ARM_LOW_LEFT_POSITION = 0.6;
    public static double ARM_LOW_RIGHT_POSITION = 0.4;
    public static double ARM_CAPSTONE_PICKUP_LEFT_POSITION = 0.70;
    public static double ARM_CAPSTONE_PICKUP_RIGHT_POSITION = 0.30;
    public static double ARM_CAPSTONE_UP_LEFT_POSITION = 0.57;
    public static double ARM_CAPSTONE_UP_RIGHT_POSITION = 0.43;
    public static double ARM_CAPSTONE_PLACE_LEFT_POSITION = 0.65;
    public static double ARM_CAPSTONE_PLACE_RIGHT_POSITION = 0.35;

    public static double TRAPDOOR_CLOSED_POSITION = 0.45;
    public static double TRAPDOOR_LOW_POSITION = 0.67;
    public static double TRAPDOOR_OPEN_POSITION = 0.7;

    public ArmMode armMode;
    public TrapdoorMode trapdoorMode;

    private ArmMode prevArmMode;
    private TrapdoorMode prevTrapdoorMode;

    private Servo armServoLeft, armServoRight;
    private Servo trapdoorServo;

    Arm(HardwareMap hardwareMap, Robot robot) {
        armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
        armServoRight = hardwareMap.get(Servo.class, "armServoRight");
        trapdoorServo = hardwareMap.get(Servo.class, "trapdoorServo");

        armMode = ArmMode.FRONT;
        trapdoorMode = TrapdoorMode.OPEN;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;

        if(armMode != prevArmMode) {
            switch (armMode) {
                case FRONT:
                    armServoLeft.setPosition(ARM_FRONT_LEFT_POSITION);
                    armServoRight.setPosition(ARM_FRONT_RIGHT_POSITION);
                    break;
                case UP:
                    armServoLeft.setPosition(ARM_UP_LEFT_POSITION);
                    armServoRight.setPosition(ARM_UP_RIGHT_POSITION);
                    break;
                case HIGH:
                    armServoLeft.setPosition(ARM_HIGH_LEFT_POSITION);
                    armServoRight.setPosition(ARM_HIGH_RIGHT_POSITION);
                    break;
                case LOW:
                    armServoLeft.setPosition(ARM_LOW_LEFT_POSITION);
                    armServoRight.setPosition(ARM_LOW_RIGHT_POSITION);
                    break;
                case CAPSTONE_PICKUP:
                    armServoLeft.setPosition(ARM_CAPSTONE_PICKUP_LEFT_POSITION);
                    armServoRight.setPosition(ARM_CAPSTONE_PICKUP_RIGHT_POSITION);
                    break;
                case CAPSTONE_UP:
                    armServoLeft.setPosition(ARM_CAPSTONE_UP_LEFT_POSITION);
                    armServoRight.setPosition(ARM_CAPSTONE_UP_RIGHT_POSITION);
                    break;
                case CAPSTONE_PLACE:
                    armServoLeft.setPosition(ARM_CAPSTONE_PLACE_LEFT_POSITION);
                    armServoRight.setPosition(ARM_CAPSTONE_PLACE_RIGHT_POSITION);
                    break;
            }
        }
        prevArmMode = armMode;

        if(trapdoorMode != prevTrapdoorMode) {
            switch (trapdoorMode) {
                case CLOSED:
                    trapdoorServo.setPosition(TRAPDOOR_CLOSED_POSITION);
                    break;
                case LOW:
                    trapdoorServo.setPosition(TRAPDOOR_LOW_POSITION);
                    break;
                case OPEN:
                    trapdoorServo.setPosition(TRAPDOOR_OPEN_POSITION);
                    break;
            }
        }
        prevTrapdoorMode = trapdoorMode;
    }
}
