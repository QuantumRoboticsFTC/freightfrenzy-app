package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.CapstoneArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.HorizontalArm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.IntakeCarousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.StickyGamepad;

public abstract class BaseTeleOP extends OpMode {
    enum DriveMode {
        NORMAL,
        SLOW,
        SUPER_SLOW
    }

    enum OuttakeTarget {
        HIGH,
        MID,
        LOW,
        SHARED
    }

    Robot robot;
    DriveMode driveMode;
    StickyGamepad stickyGamepad1 = null;
    StickyGamepad stickyGamepad2 = null;

    MultipleTelemetry telemetry;

    abstract Alliance getAlliance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(this, false, getAlliance());

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        driveMode = DriveMode.NORMAL;

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

//    ElapsedTime intakeElementTimer = new ElapsedTime(0);
    ElapsedTime intakeUpTimer = new ElapsedTime(0);
    ElapsedTime elevatorUpTimer = new ElapsedTime(0);
    ElapsedTime elevatorDownTimer = new ElapsedTime(0);
    ElapsedTime dropTimer = new ElapsedTime(0);

    boolean joyStickControl = false;
    boolean prevIntakeHasElementFront = false;
    boolean prevIntakeHasElementRear = false;

    Elevator.ElevatorMode prevElevatorMode;
    OuttakeTarget outtakeTarget = OuttakeTarget.HIGH;

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        //region Driver 1 controls

        if(!robot.drivetrain.isBusy()) {
            switch (driveMode) {
                case NORMAL:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 1, getAlliance() == Alliance.BLUE);
                    break;
                case SLOW:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 0.7, getAlliance() == Alliance.BLUE);
                    break;
                case SUPER_SLOW:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 0.5, getAlliance() == Alliance.BLUE);
                    break;
            }
        }
        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SUPER_SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }
        if(gamepad1.left_stick_button) {
            if (gamepad1.left_stick_y < -0.1) {
                if (getAlliance() == Alliance.RED) {
                    robot.intakeCarousel.rearButterflyRotation = IntakeCarousel.ButterflyRotation.DOWN;
                    robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.BUTTERFLY;
                }
                else {
                    robot.intakeCarousel.frontButterflyRotation = IntakeCarousel.ButterflyRotation.DOWN;
                    robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.BUTTERFLY;
                }
            }
            else if(gamepad1.left_stick_y > 0.1) {
                if (getAlliance() == Alliance.RED) {
                    robot.intakeCarousel.frontButterflyRotation = IntakeCarousel.ButterflyRotation.DOWN;
                    robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.BUTTERFLY;
                }
                else {
                    robot.intakeCarousel.rearButterflyRotation = IntakeCarousel.ButterflyRotation.DOWN;
                    robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.BUTTERFLY;
                }
            }
        }
        else {
            robot.intakeCarousel.frontButterflyRotation = IntakeCarousel.ButterflyRotation.UP;
            robot.intakeCarousel.rearButterflyRotation = IntakeCarousel.ButterflyRotation.UP;
            if(robot.intakeCarousel.frontIntakeMode == IntakeCarousel.IntakeMode.BUTTERFLY) {
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            if(robot.intakeCarousel.rearIntakeMode == IntakeCarousel.IntakeMode.BUTTERFLY) {
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }


        if (stickyGamepad1.right_trigger_button) {
            if(getAlliance() == Alliance.RED) {
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            else {
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }
        else if(stickyGamepad1.left_trigger_button) {
            if(getAlliance() == Alliance.RED) {
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            else {
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }
        else if(stickyGamepad1.right_trigger_button_release || stickyGamepad1.left_trigger_button_release) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            intakeUpTimer.reset();
            joyStickControl = false;
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        }

        if(gamepad1.dpad_down)
            robot.capstoneArm.armmode = CapstoneArm.ArmMode.COLLECT;
        if(stickyGamepad1.dpad_up) {
            if(robot.capstoneArm.armmode == CapstoneArm.ArmMode.DOUBLE_SCORE)
                robot.capstoneArm.armmode = CapstoneArm.ArmMode.SCORE;
            else if(robot.capstoneArm.armmode == CapstoneArm.ArmMode.SCORE)
                robot.capstoneArm.armmode = CapstoneArm.ArmMode.DOUBLE_SCORE;
            else
                robot.capstoneArm.armmode = CapstoneArm.ArmMode.SCORE;

        }
        if(gamepad1.a)
            robot.capstoneArm.clawMode = CapstoneArm.ClawMode.CLOSED;
        if(gamepad1.b)
            robot.capstoneArm.clawMode = CapstoneArm.ClawMode.OPEN;

        //endregion

        // region Driver 2 controls

        if (gamepad2.left_stick_y < -0.1) {
            if(robot.intakeCarousel.frontIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
            }
            if(robot.intakeCarousel.rearIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
            }
            joyStickControl = true;
        } else if (gamepad2.left_stick_y > 0.1) {
            if (gamepad2.left_stick_button) {
                if(robot.intakeCarousel.frontIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                    robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT;
                }
                if(robot.intakeCarousel.rearIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                    robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT;
                }
            }
            else {
                if(robot.intakeCarousel.frontIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                    robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT_SLOW;
                }
                if(robot.intakeCarousel.rearIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                    robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT_SLOW;
                }
            }
            joyStickControl = true;
        } else if (joyStickControl) {
            if(robot.intakeCarousel.frontIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                if(robot.intakeCarousel.frontIntakeMode != IntakeCarousel.IntakeMode.IN) {
                    robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
                }
            }
            if(robot.intakeCarousel.rearIntakeRotation == IntakeCarousel.IntakeRotation.DOWN) {
                if(robot.intakeCarousel.rearIntakeMode != IntakeCarousel.IntakeMode.IN) {
                    robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
                }
            }
            joyStickControl = false;
        }

        if (stickyGamepad2.a) {
            if(getAlliance() == Alliance.RED) {
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            else {
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }
        else if(stickyGamepad2.y) {
            if(getAlliance() == Alliance.RED) {
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            else {
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }
        else if (stickyGamepad2.b) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            intakeUpTimer.reset();
            joyStickControl = false;
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        }

        if(0.4 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 0.55) {
            if(robot.intakeCarousel.frontIntakeRotation == IntakeCarousel.IntakeRotation.UP) {
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT;
            }
            if(robot.intakeCarousel.rearIntakeRotation == IntakeCarousel.IntakeRotation.UP) {
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT;
            }
        }

        if(1.5 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 1.65) {
            if(robot.intakeCarousel.frontIntakeMode == IntakeCarousel.IntakeMode.OUT) {
                robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
            if(robot.intakeCarousel.rearIntakeMode == IntakeCarousel.IntakeMode.OUT) {
                robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            }
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
            if (stickyGamepad2.dpad_up) {
                outtakeTarget = OuttakeTarget.HIGH;
            } else if (stickyGamepad2.dpad_left) {
                outtakeTarget = OuttakeTarget.MID;
            } else if (stickyGamepad2.dpad_down) {
                outtakeTarget = OuttakeTarget.LOW;
            } else if (stickyGamepad2.dpad_right) {
                outtakeTarget = OuttakeTarget.SHARED;
            }
        }

        if (stickyGamepad2.right_bumper) {
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
            switch (outtakeTarget) {
                case HIGH:
                    robot.elevator.targetHeight = Elevator.TargetHeight.HIGH;
                    break;
                case MID:
                    robot.elevator.targetHeight = Elevator.TargetHeight.MID;
                    break;
                case LOW:
                case SHARED:
                    robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
                    break;
            }
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;

            if(outtakeTarget == OuttakeTarget.SHARED) {
                robot.arm.armMode = Arm.ArmMode.SHARED;
                switch(robot.horizontalArm.linkageMode) {
                    case SHARED_NEAR:
                        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.SHARED_MID;
                        break;
                    case SHARED_MID:
                        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.SHARED_FAR;
                        break;
                    case SHARED_FAR:
                    default:
                        robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.SHARED_NEAR;
                        break;
                }
            }
            else if(outtakeTarget == OuttakeTarget.LOW) {
                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.LOW;
                robot.arm.armMode = Arm.ArmMode.UP;
                elevatorUpTimer.reset();
            }
            else {
                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.NORMAL;
                robot.arm.armMode = Arm.ArmMode.UP;
                elevatorUpTimer.reset();
            }
        }
        else if (stickyGamepad2.left_bumper) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            robot.arm.armMode = Arm.ArmMode.UP;
            elevatorDownTimer.reset();
        }

        if(0.3 < elevatorDownTimer.seconds() && elevatorDownTimer.seconds() < 0.45) {
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
        }

        if(0.6 < elevatorDownTimer.seconds() && elevatorDownTimer.seconds() < 0.75) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            robot.arm.armMode = Arm.ArmMode.FRONT;
        }

        if (gamepad2.right_trigger > 0.1) {
            if(robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
                prevElevatorMode = robot.elevator.elevatorMode;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
            robot.elevator.manualPower = gamepad2.right_trigger * 0.5;
        } else if (gamepad2.left_trigger > 0.1) {
            if(robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
                prevElevatorMode = robot.elevator.elevatorMode;
            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
            robot.elevator.manualPower = gamepad2.left_trigger * (-0.07);
        } else {
            if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
                if(prevElevatorMode == Elevator.ElevatorMode.DOWN) {
                    robot.elevator.reset();
                }
                else {
                    robot.elevator.offsetPosition = robot.elevator.getCurrentHeight() - robot.elevator.targetHeight.getHeight();
                }
                robot.elevator.elevatorMode = prevElevatorMode;
                prevElevatorMode = null;
            }
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP) {
            if(stickyGamepad2.back) {
                robot.arm.armMode = Arm.ArmMode.FRONT;
                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.SAFETY_PUSH;
                robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            }
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP || robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL
         || robot.horizontalArm.linkageMode == HorizontalArm.LinkageMode.MANUAL || robot.horizontalArm.linkageMode == HorizontalArm.LinkageMode.SAFETY_PUSH) {
            if (gamepad2.right_stick_y > 0.1) {
                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.MANUAL;
                robot.horizontalArm.manualOffset -= 0.0125;
            }

            if (gamepad2.right_stick_y < -0.1) {
                robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.MANUAL;
                robot.horizontalArm.manualOffset += 0.0125;
            }
        }

        if (stickyGamepad2.x && robot.arm.armMode != Arm.ArmMode.FRONT) {
            if(outtakeTarget == OuttakeTarget.SHARED) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            }
            else if(outtakeTarget == OuttakeTarget.LOW) {
                robot.arm.armMode = Arm.ArmMode.LOW;
                dropTimer.reset();
            }
            else {
                robot.arm.armMode = Arm.ArmMode.HIGH;
                dropTimer.reset();
            }
        }

        if(outtakeTarget == OuttakeTarget.LOW) {
            if(0.4 < dropTimer.seconds() && dropTimer.seconds() < 0.55) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.LOW;
            }
        }
        else {
            if (0.2 < dropTimer.seconds() && dropTimer.seconds() < 0.35) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            }
        }

        if(stickyGamepad2.back /*stickyGamepad2.touchpad*/) {
            robot.intakeCarousel.spin();
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.CAROUSEL;
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.CAROUSEL;
        }
        else if(!gamepad2.back /*!gamepad2.touchpad*/) {
            robot.intakeCarousel.stopSpin();
            if(robot.intakeCarousel.frontIntakeMode == IntakeCarousel.IntakeMode.CAROUSEL) {
                robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            }
            if(robot.intakeCarousel.rearIntakeMode == IntakeCarousel.IntakeMode.CAROUSEL) {
                robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            }
        }

        if(robot.intakeCarousel.hasElementFront() && !prevIntakeHasElementFront) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }

        prevIntakeHasElementFront = robot.intakeCarousel.hasElementFront();

        if(robot.intakeCarousel.hasElementRear() && !prevIntakeHasElementRear) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }

        prevIntakeHasElementRear = robot.intakeCarousel.hasElementRear();

        // endregion

        telemetry.addData("Outtake Target", outtakeTarget);
        telemetry.addData("Intake mode", robot.intakeCarousel.frontIntakeMode);
        telemetry.addData("Intake Front sensor", robot.intakeCarousel.frontSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Outtake sensor", robot.arm.sensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Intake Rear sensor", robot.intakeCarousel.rearSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Elevator mode", robot.elevator.elevatorMode);
        telemetry.addData("Elevator target height", robot.elevator.targetHeight);
        telemetry.addData("Elevator current height", robot.elevator.getCurrentHeight());
        telemetry.addData("Elevator isBusy", robot.elevator.isBusy());
        telemetry.addData("Elevator power", robot.elevator.motorRight.getPower());
        telemetry.addData("Linkage manualOffset", robot.horizontalArm.manualOffset);
        telemetry.addData("Hub 1 current", robot.hub1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Hub 2 current", robot.hub2.getCurrent(CurrentUnit.AMPS));
        hardwareMap.getAll(DcMotorEx.class).forEach(motor -> {
            telemetry.addData(hardwareMap.getNamesOf(motor).toArray()[0] + " current", motor.getCurrent(CurrentUnit.AMPS));
        });
        telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
