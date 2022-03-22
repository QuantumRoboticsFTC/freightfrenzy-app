package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OdometryRetract implements Subsystem {
    public static double DOWN_POSITION = 0.525;
    public static double UP_POSITION = 0.15;

    public boolean down;

    private Servo servo;

    OdometryRetract(HardwareMap hardwareMap, boolean isAutonomous) {
        servo = hardwareMap.get(Servo.class, "odometryServo");

        down = isAutonomous;
    }

    private boolean prevDown;

    @Override
    public void update() {
//        if(down != prevDown) {
            if (down) {
                servo.setPosition(DOWN_POSITION);
            } else {
                servo.setPosition(UP_POSITION);
            }
//        }
        prevDown = down;
    }
}
