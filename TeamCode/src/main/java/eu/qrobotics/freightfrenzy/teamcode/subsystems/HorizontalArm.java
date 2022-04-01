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
        NORMAL,
        FULL,
        MANUAL
    }

    public static double LINKAGE_IN_LEFT_POSITION = 0.7;
    public static double LINKAGE_IN_RIGHT_POSITION = 0.28;
    public static double LINKAGE_NORMAL_LEFT_POSITION = 0.48;
    public static double LINKAGE_NORMAL_RIGHT_POSITION = 0.50;
    public static double LINKAGE_AUTO_MID_LEFT_POSITION = 0.45;
    public static double LINKAGE_AUTO_MID_RIGHT_POSITION = 0.53;
    public static double LINKAGE_LOW_LEFT_POSITION = 0.54;
    public static double LINKAGE_LOW_RIGHT_POSITION = 0.44;
    public static double LINKAGE_FULL_LEFT_POSITION = 0.45;
    public static double LINKAGE_FULL_RIGHT_POSITION = 0.53;

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
                case FULL:
                    horizontalArmServoLeft.setPosition(LINKAGE_FULL_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_FULL_RIGHT_POSITION);
                    manualOffset = LINKAGE_IN_LEFT_POSITION - LINKAGE_FULL_LEFT_POSITION;
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
