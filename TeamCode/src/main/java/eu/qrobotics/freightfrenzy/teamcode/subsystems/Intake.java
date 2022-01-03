package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem {

    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW
    }

    public enum IntakeRotation {
        UP,
        DOWN
    }

    public enum IntakeBlockerRampMode {
        BLOCK,
        RAMP
    }

    public static double INTAKE_IN_SPEED = 0.9;
    public static double INTAKE_IN_SLOW_SPEED = 0.2;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public static double INTAKE_UP_POSITION_LEFT = 0.865;
    public static double INTAKE_UP_POSITION_RIGHT = 0.185;
    public static double INTAKE_DOWN_POSITION_LEFT = 0.405;
    public static double INTAKE_DOWN_POSITION_RIGHT = 0.645;

    public static double INTAKE_BLOCKER_BLOCK_POSITION = 0.45;
    public static double INTAKE_BLOCKER_RAMP_POSITION = 0.67;

    public IntakeMode intakeMode;
    public IntakeRotation intakeRotation;
    public IntakeBlockerRampMode intakeBlockerRampMode;

    private DcMotor intakeMotor;
    private Servo intakePivotServoLeft;
    private Servo intakePivotServoRight;
    private Servo intakeBlockerRampServo;

    Intake(HardwareMap hardwareMap, Robot robot) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakePivotServoLeft = hardwareMap.get(Servo.class, "intakePivotServoLeft");
        intakePivotServoRight = hardwareMap.get(Servo.class, "intakePivotServoRight");
        intakeBlockerRampServo = hardwareMap.get(Servo.class, "intakeBlockerRampServo");

        intakeMode = IntakeMode.IDLE;
        intakeRotation = IntakeRotation.UP;
        intakeBlockerRampMode = IntakeBlockerRampMode.BLOCK;
    }

    public static boolean IS_DISABLED = false;

    @Override
    public void update() {
        if(IS_DISABLED) return;
        switch (intakeMode) {
            case IN:
                intakeMotor.setPower(INTAKE_IN_SPEED);
                break;
            case IN_SLOW:
                intakeMotor.setPower(INTAKE_IN_SLOW_SPEED);
                break;
            case IDLE:
                intakeMotor.setPower(INTAKE_IDLE_SPEED);
                break;
            case OUT:
                intakeMotor.setPower(INTAKE_OUT_SPEED);
                break;
            case OUT_SLOW:
                intakeMotor.setPower(INTAKE_OUT_SLOW_SPEED);
                break;
        }
        switch (intakeRotation) {
            case UP:
                intakePivotServoLeft.setPosition(INTAKE_UP_POSITION_LEFT);
                intakePivotServoRight.setPosition(INTAKE_UP_POSITION_RIGHT);
                break;
            case DOWN:
                intakePivotServoLeft.setPosition(INTAKE_DOWN_POSITION_LEFT);
                intakePivotServoRight.setPosition(INTAKE_DOWN_POSITION_RIGHT);
                break;
        }
        switch (intakeBlockerRampMode) {
            case BLOCK:
                intakeBlockerRampServo.setPosition(INTAKE_BLOCKER_BLOCK_POSITION);
                break;
            case RAMP:
                intakeBlockerRampServo.setPosition(INTAKE_BLOCKER_RAMP_POSITION);
                break;
        }
    }
}
