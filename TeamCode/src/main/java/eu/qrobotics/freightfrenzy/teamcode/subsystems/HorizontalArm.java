package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HorizontalArm implements Subsystem {

    public enum LinkageMode {
        IN,
        LOW,
        AUTO_MID,
        AUTO_HIGH,
        NORMAL,
        FULL,
        CAPSTONE_PICKUP,
        CAPSTONE_PLACE,
        SAFETY_PUSH,
        MANUAL
    }

    public static double LINKAGE_IN_LEFT_POSITION = 0.72;
    public static double LINKAGE_IN_RIGHT_POSITION = 0.26;
    public static double LINKAGE_NORMAL_LEFT_POSITION = 0.49;
    public static double LINKAGE_NORMAL_RIGHT_POSITION = 0.49;
    public static double LINKAGE_AUTO_MID_LEFT_POSITION = 0.45;
    public static double LINKAGE_AUTO_MID_RIGHT_POSITION = 0.53;
    public static double LINKAGE_AUTO_HIGH_LEFT_POSITION = 0.5;
    public static double LINKAGE_AUTO_HIGH_RIGHT_POSITION = 0.48;
    public static double LINKAGE_LOW_LEFT_POSITION = 0.55;
    public static double LINKAGE_LOW_RIGHT_POSITION = 0.43;
    public static double LINKAGE_FULL_LEFT_POSITION = 0.45;
    public static double LINKAGE_FULL_RIGHT_POSITION = 0.53;
    public static double LINKAGE_CAPSTONE_PICKUP_LEFT_POSITION = 0.60;
    public static double LINKAGE_CAPSTONE_PICKUP_RIGHT_POSITION = 0.38;
    public static double LINKAGE_CAPSTONE_PLACE_LEFT_POSITION = 0.54;
    public static double LINKAGE_CAPSTONE_PLACE_RIGHT_POSITION = 0.44;
    public static double LINKAGE_SAFETY_PUSH_LEFT_POSITION = 0.6;
    public static double LINKAGE_SAFETY_PUSH_RIGHT_POSITION = 0.38;

    public LinkageMode linkageMode;

    private LinkageMode prevArmMode;

    public double manualOffset = 0;

    private Servo horizontalArmServoLeft, horizontalArmServoRight;
    HorizontalArm(HardwareMap hardwareMap, Robot robot) {
        horizontalArmServoLeft = hardwareMap.get(Servo.class, "horizontalArmServoLeft");
        horizontalArmServoRight = hardwareMap.get(Servo.class, "horizontalArmServoRight");

        linkageMode = LinkageMode.IN;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;

        if(linkageMode != prevArmMode || linkageMode == LinkageMode.MANUAL) {
            switch (linkageMode) {
                case IN:
                    horizontalArmServoLeft.setPosition(LINKAGE_IN_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_IN_RIGHT_POSITION);
                    manualOffset = 0.0;
                    break;
                case LOW:
                    horizontalArmServoLeft.setPosition(LINKAGE_LOW_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_LOW_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_LOW_LEFT_POSITION;
                    break;
                case NORMAL:
                    horizontalArmServoLeft.setPosition(LINKAGE_NORMAL_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_NORMAL_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_NORMAL_LEFT_POSITION;
                    break;
                case AUTO_MID:
                    horizontalArmServoLeft.setPosition(LINKAGE_AUTO_MID_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_AUTO_MID_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_AUTO_MID_LEFT_POSITION;
                    break;
                case AUTO_HIGH:
                    horizontalArmServoLeft.setPosition(LINKAGE_AUTO_HIGH_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_AUTO_HIGH_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_AUTO_HIGH_LEFT_POSITION;
                    break;
                case FULL:
                    horizontalArmServoLeft.setPosition(LINKAGE_FULL_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_FULL_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_FULL_LEFT_POSITION;
                    break;
                case CAPSTONE_PICKUP:
                    horizontalArmServoLeft.setPosition(LINKAGE_CAPSTONE_PICKUP_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_CAPSTONE_PICKUP_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_CAPSTONE_PICKUP_LEFT_POSITION;
                    break;
                case CAPSTONE_PLACE:
                    horizontalArmServoLeft.setPosition(LINKAGE_CAPSTONE_PLACE_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_CAPSTONE_PLACE_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_CAPSTONE_PLACE_LEFT_POSITION;
                    break;
                case SAFETY_PUSH:
                    horizontalArmServoLeft.setPosition(LINKAGE_SAFETY_PUSH_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_SAFETY_PUSH_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_SAFETY_PUSH_LEFT_POSITION;
                    break;
                case MANUAL:
                    horizontalArmServoLeft.setPosition(LINKAGE_IN_LEFT_POSITION - manualOffset);
                    horizontalArmServoRight.setPosition(LINKAGE_IN_RIGHT_POSITION + manualOffset);
                    break;
            }
        }
        prevArmMode = linkageMode;
    }
}
