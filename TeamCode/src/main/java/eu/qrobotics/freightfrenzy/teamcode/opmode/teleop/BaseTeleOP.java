package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Capstone;
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

        robot.elevator.targetHeight = Elevator.TargetHeight.HIGH;

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
    ElapsedTime elevatorDownTimer = new ElapsedTime(0);

//    boolean elevatorUpToggle = true;
//    boolean elevatorDownToggle = true;

//    boolean keepCapstoneMode = true;
    boolean joyStickControl = false;

//    Elevator.ElevatorMode prevElevatorMode;

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        //region Driver 1 controls

        if(!robot.drivetrain.isBusy()) {
            switch (driveMode) {
                case NORMAL:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 1);
                    break;
                case SLOW:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 0.7);
                    break;
                case SUPER_SLOW:
                    robot.drivetrain.setMotorPowersFromGamepad(gamepad1, 0.5);
                    break;
            }
        }
        if (stickyGamepad1.left_bumper) {
            driveMode = DriveMode.SUPER_SLOW;
        }
        if (stickyGamepad1.right_bumper) {
            driveMode = DriveMode.NORMAL;
        }

//        if(gamepad1.dpad_up) {
//            robot.capstone.tiltDirection = Capstone.TiltDirection.UP;
//        } else if (gamepad1.dpad_down) {
//            robot.capstone.tiltDirection = Capstone.TiltDirection.DOWN;
//        } else {
//            robot.capstone.tiltDirection = Capstone.TiltDirection.IDLE;
//        }
//
//        if(gamepad1.dpad_left) {
//            robot.capstone.swivelDirection = Capstone.SwivelDirection.LEFT;
//        } else if (gamepad1.dpad_right) {
//            robot.capstone.swivelDirection = Capstone.SwivelDirection.RIGHT;
//        } else {
//            robot.capstone.swivelDirection = Capstone.SwivelDirection.IDLE;
//        }

//        if(gamepad1.x) {
//            robot.capstone.extendMode = Capstone.ExtendMode.EXTEND;
//            keepCapstoneMode = true;
//            Capstone.CAPSTONE_TILT_POSITION = 0.8;
//        }
//        if(gamepad1.y) {
//            robot.capstone.extendMode = Capstone.ExtendMode.RETRACT;
//            keepCapstoneMode = true;
//        }
//        if(gamepad1.a) {
//            robot.capstone.extendMode = Capstone.ExtendMode.EXTEND;
//            keepCapstoneMode = false;
//        } else if (gamepad1.b) {
//            robot.capstone.extendMode = Capstone.ExtendMode.RETRACT;
//            keepCapstoneMode = false;
//        } else if (!keepCapstoneMode) {
//            robot.capstone.extendMode = Capstone.ExtendMode.IDLE;
//        }

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
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            joyStickControl = false;
        }

        if (stickyGamepad2.a) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IN;
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
        }
        else if(stickyGamepad2.y) {
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.DOWN;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IN;
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
        }
        else if (stickyGamepad2.b) {
            robot.intakeCarousel.frontIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            robot.intakeCarousel.rearIntakeRotation = IntakeCarousel.IntakeRotation.UP;
            intakeUpTimer.reset();
            joyStickControl = false;
        }

        if(1.0 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 1.1) {
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.OUT;
        }

        if(2.0 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 2.1) {
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
            if (stickyGamepad2.dpad_up) {
                robot.elevator.targetHeight = Elevator.TargetHeight.HIGH;
            } else if (stickyGamepad2.dpad_left) {
                robot.elevator.targetHeight = Elevator.TargetHeight.MID;
            } else if (stickyGamepad2.dpad_down) {
                robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
            } else if (stickyGamepad2.dpad_right) {
                robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
            }
        }

        if (stickyGamepad2.right_bumper) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.MID;
            robot.arm.armMode = Arm.ArmMode.HIGH;
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.IDLE;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.IDLE;
//            elevatorUpToggle = false;
        }
        else if (stickyGamepad2.left_bumper) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
            robot.horizontalArm.linkageMode = HorizontalArm.LinkageMode.IN;
            robot.arm.armMode = Arm.ArmMode.FRONT;
            elevatorDownTimer.reset();
        }

//        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP && !elevatorUpToggle && robot.elevator.getDistanceLeft() < 3.0) {
//            robot.arm.armMode = Arm.ArmMode.BACK;
//            elevatorUpToggle = true;
//        }

        if(0.6 < elevatorDownTimer.seconds() && elevatorDownTimer.seconds() < 0.7) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
//            elevatorDownToggle = false;
        }

//        if (gamepad2.right_trigger > 0.1) {
//            if(robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
//                prevElevatorMode = robot.elevator.elevatorMode;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
//            robot.elevator.manualPower = gamepad2.right_trigger * 0.3;
//        } else if (gamepad2.left_trigger > 0.1) {
//            if(robot.elevator.elevatorMode != Elevator.ElevatorMode.MANUAL)
//                prevElevatorMode = robot.elevator.elevatorMode;
//            robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
//            robot.elevator.manualPower = gamepad2.left_trigger * (-0.1);
//        } else {
//            if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
//                if(prevElevatorMode == Elevator.ElevatorMode.DOWN) {
//                    robot.elevator.reset();
//                }
//                else {
//                    robot.elevator.offsetPosition = robot.elevator.getCurrentHeight() - robot.elevator.targetHeight.getHeight();
//                }
//                robot.elevator.elevatorMode = prevElevatorMode;
//                prevElevatorMode = null;
//            }
//        }

        if (stickyGamepad2.x && robot.arm.armMode != Arm.ArmMode.FRONT) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
        }

        if(stickyGamepad2.back) {
            robot.intakeCarousel.spin();
            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
            robot.intakeCarousel.rearIntakeMode = IntakeCarousel.IntakeMode.CAROUSEL;
        }
        else if(!gamepad2.back) {
            robot.intakeCarousel.stopSpin();
        }

//        if(!robot.intakeCarousel.hasElement() || robot.intakeCarousel.frontIntakeMode != IntakeCarousel.IntakeMode.IN || !elevatorDownToggle || robot.elevator.elevatorMode != Elevator.ElevatorMode.DOWN) {
//            intakeElementTimer.reset();
//        }
//
//        if(0.5 < intakeElementTimer.seconds() && intakeElementTimer.seconds() < 0.6) {
//            robot.intakeCarousel.frontIntakeMode = IntakeCarousel.IntakeMode.OUT;
//            joyStickControl = false;
//        }

        // endregion

        telemetry.addData("Intake current (mA)", robot.intakeCarousel.frontIntakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Intake mode", robot.intakeCarousel.frontIntakeMode);
//        telemetry.addData("Intake sensor", robot.intakeCarousel.intakeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Elevator mode", robot.elevator.elevatorMode);
        telemetry.addData("Elevator target height", robot.elevator.targetHeight);
        telemetry.addData("Elevator current height", robot.elevator.getCurrentHeight());
        telemetry.addData("Elevator isBusy", robot.elevator.isBusy());
        telemetry.addData("Elevator power", robot.elevator.motorRight.getPower());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
