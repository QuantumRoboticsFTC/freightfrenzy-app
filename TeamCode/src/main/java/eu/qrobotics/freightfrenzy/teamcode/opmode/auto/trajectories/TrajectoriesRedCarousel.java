package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.*;

public class TrajectoriesRedCarousel {
    public static Pose2d START_POSE = new Pose2d(-40, -65, Math.toRadians(0));

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectoriesA() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-47, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-24, -38, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-63, -54, Math.toRadians(-80)), Math.toRadians(-80))
                .forward(1, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-60, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45))
                .lineToConstantHeading(new Vector2d(-65, -57))
                .splineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(-30)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-20.5, -37, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-64, -34, Math.toRadians(0)))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-36, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-22, -39, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-63, -54, Math.toRadians(-80)), Math.toRadians(-80))
                .forward(1, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-60, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45))
                .lineToConstantHeading(new Vector2d(-65, -57))
                .splineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(-30)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-21, -37, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-64, -34, Math.toRadians(0)))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-28, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-24, -38, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-63, -54, Math.toRadians(-80)), Math.toRadians(-80))
                .forward(1, SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-60, -40))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45))
                .lineToConstantHeading(new Vector2d(-65, -57))
                .splineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(-30)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-20.5, -37, Math.toRadians(-135)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-64, -34, Math.toRadians(0)))
                .build()
        );

        return trajectories;
    }
}