package eu.qrobotics.freightfrenzy.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake implements Subsystem {

    public enum IntakeMode {
        IN,
        IN_SLOW,
        IDLE,
        OUT,
        OUT_SLOW
    }

    public static double INTAKE_IN_SPEED = 1.0;
    public static double INTAKE_IN_SLOW_SPEED = 0.7;
    public static double INTAKE_IDLE_SPEED = 0;
    public static double INTAKE_OUT_SPEED = -1.0;
    public static double INTAKE_OUT_SLOW_SPEED = -0.4;

    public IntakeMode intakeMode;

    private IntakeMode prevIntakeMode;

    public DcMotorEx intakeMotor;
    public ColorRangeSensor intakeSensor;

    Intake(HardwareMap hardwareMap, Robot robot) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakeSensor");

        intakeMode = IntakeMode.IDLE;
    }

    public static boolean IS_DISABLED = false;

    public boolean hasElement() {
        return intakeSensor.getDistance(DistanceUnit.CM) < 45;
    }

    @Override
    public void update() {
        if(IS_DISABLED) return;

//        if(intakeMode != prevIntakeMode) {
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
//        }
        prevIntakeMode = intakeMode;
    }
}
