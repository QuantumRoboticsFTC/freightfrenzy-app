package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Capstone implements Subsystem {
    public enum ExtendMode {
        EXTEND,
        RETRACT,
        IDLE
    }

    public enum TiltDirection {
        DOWN,
        UP,
        IDLE
    }

    public enum SwivelDirection {
        LEFT,
        RIGHT,
        IDLE
    }

    public static double CAPSTONE_SWIVEL_POSITION = 0.5;
    public static double CAPSTONE_TILT_POSITION = 0.525;

    public ExtendMode extendMode;
    public TiltDirection tiltDirection;
    public SwivelDirection swivelDirection;

    private ElapsedTime deltaTime = new ElapsedTime();

    private Servo tiltServo;
    private Servo swivelServo;
    private CRServo extendServo;

    Capstone(HardwareMap hardwareMap, Robot robot) {
        tiltServo = hardwareMap.get(Servo.class, "capstoneTiltServo");
        tiltServo.setDirection(Servo.Direction.REVERSE);
        swivelServo = hardwareMap.get(Servo.class, "capstoneSwivelServo");
        extendServo = hardwareMap.get(CRServo.class, "capstoneExtendServo");

        extendMode = ExtendMode.IDLE;
        tiltDirection = TiltDirection.IDLE;
        swivelDirection = SwivelDirection.IDLE;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;

        switch(extendMode) {
            case EXTEND:
                extendServo.setPower(1);
                break;
            case RETRACT:
                extendServo.setPower(-1);
                break;
            case IDLE:
                extendServo.setPower(0);
                break;
        }

        switch(tiltDirection) {
            case DOWN:
                CAPSTONE_TILT_POSITION -= 0.1 * deltaTime.seconds();
                break;
            case UP:
                CAPSTONE_TILT_POSITION += 0.1 * deltaTime.seconds();
                break;
            case IDLE:
                break;

        }
        switch (swivelDirection) {
            case LEFT:
                CAPSTONE_SWIVEL_POSITION -= 0.05 * deltaTime.seconds();
                break;
            case RIGHT:
                CAPSTONE_SWIVEL_POSITION += 0.05 * deltaTime.seconds();
                break;
            case IDLE:
                break;
        }

        CAPSTONE_TILT_POSITION = Range.clip(CAPSTONE_TILT_POSITION, 0, 1);
        CAPSTONE_SWIVEL_POSITION = Range.clip(CAPSTONE_SWIVEL_POSITION, 0, 1);

        tiltServo.setPosition(CAPSTONE_TILT_POSITION);
        swivelServo.setPosition(CAPSTONE_SWIVEL_POSITION);

        deltaTime.reset();
    }
}
