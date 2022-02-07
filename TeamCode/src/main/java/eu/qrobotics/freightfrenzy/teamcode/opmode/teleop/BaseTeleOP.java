package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Capstone;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Intake;
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

    ElapsedTime intakeDropTimer = new ElapsedTime(0);
    ElapsedTime intakeUpTimer = new ElapsedTime(0);
    ElapsedTime intakeDownTimer = new ElapsedTime(0);
    ElapsedTime capstoneTimer = new ElapsedTime(0);
    ElapsedTime elevatorDownTimer = new ElapsedTime(0);

    boolean elevatorUpToggle = true;
    boolean elevatorDownToggle = true;

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
        if(gamepad1.right_trigger > 0.1) {
            robot.carousel.spin();
        }
        if(gamepad1.left_trigger > 0.1) {
            robot.carousel.stopSpin();
        }

        //endregion

        // region Driver 2 controls

        if (robot.intake.intakeRotation != Intake.IntakeRotation.UP) {
            if (gamepad2.left_stick_y < -0.1) {
                robot.intake.intakeMode = Intake.IntakeMode.IN;
            } else if (gamepad2.left_stick_y > 0.1) {
                if (gamepad2.left_stick_button)
                    robot.intake.intakeMode = Intake.IntakeMode.OUT;
                else
                    robot.intake.intakeMode = Intake.IntakeMode.OUT_SLOW;
            } else if (robot.intake.intakeMode == Intake.IntakeMode.OUT_SLOW || robot.intake.intakeMode == Intake.IntakeMode.OUT)
                robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }

        if (stickyGamepad2.a) {
            robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;
            robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.BLOCK;
            intakeDownTimer.reset();
        }
        else if (stickyGamepad2.b) {
            if(robot.intake.intakeRotation == Intake.IntakeRotation.UP && robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN && robot.arm.armMode == Arm.ArmMode.FRONT) {
                robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.RAMP;
                robot.intake.intakeRotation = Intake.IntakeRotation.TRANSFER;
                robot.arm.armMode = Arm.ArmMode.TRANSFER;
                robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                intakeDropTimer.reset();
            }
            else {
                robot.intake.intakeRotation = Intake.IntakeRotation.UP;
                intakeUpTimer.reset();
            }
        }

        if(0.5 < intakeDownTimer.seconds() && intakeDownTimer.seconds() < 0.6) {
            if(robot.intake.intakeRotation == Intake.IntakeRotation.DOWN) {
                robot.intake.intakeMode = Intake.IntakeMode.IN;
            }
        }

        if(0.2 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 0.3) {
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }

        if(0.4 < intakeDropTimer.seconds() && intakeDropTimer.seconds() < 0.5) {
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }
        if(1.0 < intakeDropTimer.seconds() && intakeDropTimer.seconds() < 1.1) {
            robot.intake.intakeRotation = Intake.IntakeRotation.UP;
            if(robot.arm.armMode == Arm.ArmMode.TRANSFER && robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
                robot.arm.armMode = Arm.ArmMode.FRONT;
            }
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN) {
            if (stickyGamepad2.dpad_up) {
                robot.elevator.targetHeight = Elevator.TargetHeight.HIGH;
            } else if (stickyGamepad2.dpad_left) {
                robot.elevator.targetHeight = Elevator.TargetHeight.MID;
            } else if (stickyGamepad2.dpad_down) {
                robot.elevator.targetHeight = Elevator.TargetHeight.LOW;
            } else if (stickyGamepad2.dpad_right) {
                robot.elevator.targetHeight = Elevator.TargetHeight.CAP;
            }
        }

        if (stickyGamepad2.right_bumper) {
           robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
           robot.capstone.capstoneMode = Capstone.CapstoneMode.UP_CLEARANCE;
           robot.intake.intakeRotation = Intake.IntakeRotation.UP_CLEARANCE;
           elevatorUpToggle = false;
        }
        else if (stickyGamepad2.left_bumper) {
            robot.arm.armMode = Arm.ArmMode.FRONT;
            robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
            elevatorDownTimer.reset();
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.UP && !elevatorUpToggle && robot.elevator.getDistanceLeft() < 0.5) {
            robot.arm.armMode = Arm.ArmMode.BACK;
            elevatorUpToggle = true;
        }

        if(0.5 < elevatorDownTimer.seconds() && elevatorDownTimer.seconds() < 0.6) {
            robot.elevator.elevatorMode = Elevator.ElevatorMode.DOWN;
            elevatorDownToggle = false;
        }

        if(robot.elevator.elevatorMode == Elevator.ElevatorMode.DOWN && !elevatorDownToggle && robot.elevator.getDistanceLeft() < 0.5) {
            robot.capstone.capstoneMode = Capstone.CapstoneMode.UP;
            if(robot.intake.intakeRotation == Intake.IntakeRotation.UP_CLEARANCE) {
                robot.intake.intakeRotation = Intake.IntakeRotation.UP;
            }
            elevatorDownToggle = true;
        }

        if (robot.elevator.elevatorMode == Elevator.ElevatorMode.UP ||
                robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
            if (gamepad2.right_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                robot.elevator.manualPower = gamepad2.right_trigger * 0.3;
            } else if (gamepad2.left_trigger > 0.1) {
                robot.elevator.elevatorMode = Elevator.ElevatorMode.MANUAL;
                robot.elevator.manualPower = gamepad2.left_trigger * (-0.1);
            } else {
                if (robot.elevator.elevatorMode == Elevator.ElevatorMode.MANUAL) {
                    robot.elevator.elevatorMode = Elevator.ElevatorMode.UP;
                    robot.elevator.offsetPosition = robot.elevator.getCurrentHeight() - robot.elevator.targetHeight.getHeight();
                }
            }
        }

        if (stickyGamepad2.x && robot.arm.armMode == Arm.ArmMode.BACK) {
            if(robot.elevator.targetHeight == Elevator.TargetHeight.CAP) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
            }
            else {
                if (robot.arm.trapdoorMode != Arm.TrapdoorMode.OPEN)
                    robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
                else
                    robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
            }
        }

        if (stickyGamepad2.y && robot.arm.armMode == Arm.ArmMode.BACK) {
            if(robot.elevator.targetHeight == Elevator.TargetHeight.CAP) {
                robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
            }
            else {
                if (robot.arm.trapdoorMode != Arm.TrapdoorMode.OPEN_REVERSE)
                    robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
                else
                    robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            }
        }

        if(stickyGamepad2.back) {
            robot.arm.armMode = Arm.ArmMode.CAPSTONE;
            robot.capstone.capstoneMode = Capstone.CapstoneMode.OUTTAKE;
            robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;
            capstoneTimer.reset();
        }

        if(0.5 < capstoneTimer.seconds() && capstoneTimer.seconds() < 0.6) {
            robot.capstone.capstoneMode = Capstone.CapstoneMode.UP;
            robot.intake.intakeRotation = Intake.IntakeRotation.UP;
        }

        // endregion

        telemetry.addData("Intake rotation", robot.intake.intakeRotation);
        telemetry.addData("Intake mode", robot.intake.intakeMode);
        telemetry.addData("Intake blocker", robot.intake.intakeBlockerRampMode);
        telemetry.addData("Elevator mode", robot.elevator.elevatorMode);
        telemetry.addData("Elevator target height", robot.elevator.targetHeight);
        telemetry.addData("Elevator current height", robot.elevator.getCurrentHeight());
        telemetry.addData("Elevator isBusy", robot.elevator.isBusy());
        telemetry.addData("Elevator power", robot.elevator.motorLeft.getPower());
//        telemetry.addData("Last PID controller error", robot.elevator.getLastControllerError());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
