package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HorizontalArm implements Subsystem {

    public enum LinkageMode {
        IN,
        MID,
        OUT
    }

    public static double LINKAGE_IN_LEFT_POSITION = 0.7;
    public static double LINKAGE_IN_RIGHT_POSITION = 0.275;
    public static double LINKAGE_MID_LEFT_POSITION = 0.48;
    public static double LINKAGE_MID_RIGHT_POSITION = 0.495;
    public static double LINKAGE_OUT_LEFT_POSITION = 0.45;
    public static double LINKAGE_OUT_RIGHT_POSITION = 0.525;

    public LinkageMode linkageMode;

    private LinkageMode prevArmMode;

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

        if(linkageMode != prevArmMode) {
            switch (linkageMode) {
                case IN:
                    horizontalArmServoLeft.setPosition(LINKAGE_IN_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_IN_RIGHT_POSITION);
                    break;
                case MID:
                    horizontalArmServoLeft.setPosition(LINKAGE_MID_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_MID_RIGHT_POSITION);
                    break;
                case OUT:
                    horizontalArmServoLeft.setPosition(LINKAGE_OUT_LEFT_POSITION);
                    horizontalArmServoRight.setPosition(LINKAGE_OUT_RIGHT_POSITION);
                    break;
            }
        }
        prevArmMode = linkageMode;
    }
}
