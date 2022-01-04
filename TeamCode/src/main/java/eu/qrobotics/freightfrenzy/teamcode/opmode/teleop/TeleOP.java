package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import eu.qrobotics.freightfrenzy.teamcode.subsystems.Arm;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Capstone;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Carousel;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Elevator;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Intake;
import eu.qrobotics.freightfrenzy.teamcode.subsystems.Robot;
import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;
import eu.qrobotics.freightfrenzy.teamcode.util.StickyGamepad;

@TeleOp
public class TeleOP extends OpMode {
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

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(this, false, Alliance.RED);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        driveMode = DriveMode.NORMAL;

        telemetry.log().add("Ready!");
    }

    @Override
    public void start() {
        robot.start();
    }

    ElapsedTime trapdoorTimer = new ElapsedTime(0);
    ElapsedTime intakeUpTimer = new ElapsedTime(0);
    ElapsedTime intakeDownTimer = new ElapsedTime(0);
    ElapsedTime capstoneTimer = new ElapsedTime(0);

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
        robot.carousel.carouselPower = gamepad1.right_trigger - gamepad1.left_trigger;

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
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
            if(robot.intake.intakeRotation == Intake.IntakeRotation.UP && robot.elevator.getElevatorMode() == Elevator.ElevatorMode.DOWN && robot.arm.armMode == Arm.ArmMode.FRONT) {
                robot.intake.intakeBlockerRampMode = Intake.IntakeBlockerRampMode.RAMP;
                robot.intake.intakeMode = Intake.IntakeMode.IN_SLOW;
                intakeUpTimer.reset();
            }
            robot.intake.intakeRotation = Intake.IntakeRotation.UP;
        }
        if(0.5 < intakeDownTimer.seconds() && intakeDownTimer.seconds() < 0.6) {
            robot.intake.intakeMode = Intake.IntakeMode.IN;
        }

        if(0.5 < intakeUpTimer.seconds() && intakeUpTimer.seconds() < 0.6) {
            robot.intake.intakeMode = Intake.IntakeMode.IDLE;
        }

        if (stickyGamepad2.dpad_up) {
            robot.elevator.setTargetHeight(Elevator.TargetHeight.HIGH);
        }
        else if (stickyGamepad2.dpad_left) {
            robot.elevator.setTargetHeight(Elevator.TargetHeight.MID);
        }
        else if (stickyGamepad2.dpad_down) {
            robot.elevator.setTargetHeight(Elevator.TargetHeight.LOW);
        }
        else if (stickyGamepad2.dpad_right) {
            robot.elevator.setTargetHeight(Elevator.TargetHeight.CAP);
        }

        if (stickyGamepad2.right_bumper) {
           robot.elevator.setElevatorMode(Elevator.ElevatorMode.UP);
           robot.arm.armMode = Arm.ArmMode.BACK;
        }
        else if (stickyGamepad2.left_bumper) {
            robot.elevator.setElevatorMode(Elevator.ElevatorMode.DOWN);
            robot.arm.armMode = Arm.ArmMode.FRONT;
        }

        if(robot.elevator.getElevatorMode() == Elevator.ElevatorMode.UP) {// || robot.elevator.getElevatorMode() == Elevator.ElevatorMode.MANUAL) {
            robot.elevator.manualPower = gamepad2.right_trigger * 0.75 - gamepad2.left_trigger * 0.25;
        }
        else {
            robot.elevator.manualPower = 0;
        }

        if (stickyGamepad2.x && robot.arm.armMode != Arm.ArmMode.FRONT) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN;
            trapdoorTimer.reset();
        }

        if(stickyGamepad2.y) {
            robot.arm.armMode = Arm.ArmMode.CAPSTONE;
            robot.capstone.capstoneMode = Capstone.CapstoneMode.OUTTAKE;
            robot.intake.intakeRotation = Intake.IntakeRotation.DOWN;
            capstoneTimer.reset();
        }

        if(0.5 < trapdoorTimer.seconds() && trapdoorTimer.seconds() < 0.6) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.OPEN_REVERSE;
        }

        if(1.0 < trapdoorTimer.seconds() && trapdoorTimer.seconds() < 1.1) {
            robot.arm.trapdoorMode = Arm.TrapdoorMode.CLOSED;
        }

        if(1.0 < capstoneTimer.seconds() && capstoneTimer.seconds() < 1.1) {
            robot.capstone.capstoneMode = Capstone.CapstoneMode.UP;
            robot.intake.intakeRotation = Intake.IntakeRotation.UP;
        }

        // endregion

        telemetry.addData("Elevator mode", robot.elevator.getElevatorMode());
        telemetry.addData("Elevator target height", robot.elevator.getTargetHeight());
        telemetry.addData("Elevator current height", robot.elevator.getCurrentHeight());
        telemetry.addData("Elevator isBusy", robot.elevator.isBusy());
        telemetry.addData("Elevator power", robot.elevator.motorLeft.getPower());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
