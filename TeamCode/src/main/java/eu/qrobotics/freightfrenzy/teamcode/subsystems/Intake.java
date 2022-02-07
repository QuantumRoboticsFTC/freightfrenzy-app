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
        TRANSFER,
        UP_CLEARANCE,
        DOWN
    }

    public enum IntakeBlockerRampMode {
        BLOCK,
        RAMP
    }

    public static double INTAKE_IN_SPEED = 1.0;
    public static double INTAKE_IN_SLOW_SPEED = 0.3;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -0.8;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public static double INTAKE_UP_POSITION_LEFT = 0.865;
    public static double INTAKE_UP_POSITION_RIGHT = 0.185;
    public static double INTAKE_TRANSFER_POSITION_LEFT = 0.900;
    public static double INTAKE_TRANSFER_POSITION_RIGHT = 0.150;
    public static double INTAKE_UP_CLEARANCE_POSITION_LEFT = 0.82;
    public static double INTAKE_UP_CLEARANCE_POSITION_RIGHT = 0.23;
    public static double INTAKE_DOWN_POSITION_LEFT = 0.405;
    public static double INTAKE_DOWN_POSITION_RIGHT = 0.645;

    public static double INTAKE_BLOCKER_BLOCK_POSITION = 0.45;
    public static double INTAKE_BLOCKER_RAMP_POSITION = 0.78;

    public IntakeMode intakeMode;
    public IntakeRotation intakeRotation;
    public IntakeBlockerRampMode intakeBlockerRampMode;

    private IntakeMode prevIntakeMode;
    private IntakeRotation prevIntakeRotation;
    private IntakeBlockerRampMode prevIntakeBlockerRampMode;

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

        if(intakeMode != prevIntakeMode) {
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
        }
        prevIntakeMode = intakeMode;

//        if(intakeRotation != prevIntakeRotation) {
            switch (intakeRotation) {
                case UP:
                    intakePivotServoLeft.setPosition(INTAKE_UP_POSITION_LEFT);
                    intakePivotServoRight.setPosition(INTAKE_UP_POSITION_RIGHT);
                    break;
                case TRANSFER:
                    intakePivotServoLeft.setPosition(INTAKE_TRANSFER_POSITION_LEFT);
                    intakePivotServoRight.setPosition(INTAKE_TRANSFER_POSITION_RIGHT);
                    break;
                case UP_CLEARANCE:
                    intakePivotServoLeft.setPosition(INTAKE_UP_CLEARANCE_POSITION_LEFT);
                    intakePivotServoRight.setPosition(INTAKE_UP_CLEARANCE_POSITION_RIGHT);
                    break;
                case DOWN:
                    intakePivotServoLeft.setPosition(INTAKE_DOWN_POSITION_LEFT);
                    intakePivotServoRight.setPosition(INTAKE_DOWN_POSITION_RIGHT);
                    break;
            }
//        }
        prevIntakeRotation = intakeRotation;

        if(intakeBlockerRampMode != prevIntakeBlockerRampMode) {
            switch (intakeBlockerRampMode) {
                case BLOCK:
                    intakeBlockerRampServo.setPosition(INTAKE_BLOCKER_BLOCK_POSITION);
                    break;
                case RAMP:
                    intakeBlockerRampServo.setPosition(INTAKE_BLOCKER_RAMP_POSITION);
                    break;
            }
        }
        prevIntakeBlockerRampMode = intakeBlockerRampMode;
    }
}
