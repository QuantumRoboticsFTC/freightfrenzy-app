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
        SHARED_NEAR,
        SHARED_MID,
        SHARED_FAR,
        SAFETY_PUSH,
        MANUAL
    }

    public static double LINKAGE_IN_LEFT_POSITION = 0.725;
    public static double LINKAGE_IN_RIGHT_POSITION = 0.245;
    public static double LINKAGE_NORMAL_LEFT_POSITION = 0.470;
    public static double LINKAGE_NORMAL_RIGHT_POSITION = 0.500;
    public static double LINKAGE_AUTO_MID_LEFT_POSITION = 0.455;
    public static double LINKAGE_AUTO_MID_RIGHT_POSITION = 0.515;
    public static double LINKAGE_AUTO_HIGH_LEFT_POSITION = 0.510;
    public static double LINKAGE_AUTO_HIGH_RIGHT_POSITION = 0.460;
    public static double LINKAGE_LOW_LEFT_POSITION = 0.48;
    public static double LINKAGE_LOW_RIGHT_POSITION = 0.49;
    public static double LINKAGE_FULL_LEFT_POSITION = 0.455;
    public static double LINKAGE_FULL_RIGHT_POSITION = 0.515;
    public static double LINKAGE_SHARED_NEAR_LEFT_POSITION = 0.725;
    public static double LINKAGE_SHARED_NEAR_RIGHT_POSITION = 0.245;
    public static double LINKAGE_SHARED_MID_LEFT_POSITION = 0.660;
    public static double LINKAGE_SHARED_MID_RIGHT_POSITION = 0.330;
    public static double LINKAGE_SHARED_FAR_LEFT_POSITION = 0.620;
    public static double LINKAGE_SHARED_FAR_RIGHT_POSITION = 0.370;
    public static double LINKAGE_SAFETY_PUSH_LEFT_POSITION = 0.605;
    public static double LINKAGE_SAFETY_PUSH_RIGHT_POSITION = 0.365;

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
                case SHARED_NEAR:
                    horizontalArmServoLeft.setPosition(LINKAGE_SHARED_NEAR_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_SHARED_NEAR_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_SHARED_NEAR_LEFT_POSITION;
                    break;
                case SHARED_MID:
                    horizontalArmServoLeft.setPosition(LINKAGE_SHARED_MID_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_SHARED_MID_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_SHARED_MID_LEFT_POSITION;
                    break;
                case SHARED_FAR:
                    horizontalArmServoLeft.setPosition(LINKAGE_SHARED_FAR_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_SHARED_FAR_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_SHARED_FAR_LEFT_POSITION;
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
