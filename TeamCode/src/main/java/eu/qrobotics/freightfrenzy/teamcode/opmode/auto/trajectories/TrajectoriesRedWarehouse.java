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

public class TrajectoriesRedWarehouse {
    public static Pose2d START_POSE = new Pose2d(12, -65, Math.toRadians(0));

    private static Pose2d getTrajectorySequenceEndPose(List<Trajectory> trajectories) {
        if(trajectories.size() == 0)
            return START_POSE;
        return trajectories.get(trajectories.size() - 1).end();
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startTangent) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPose(trajectories), startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectoriesA() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2, -50), Math.toRadians(90))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)), Math.toRadians(90))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -50), Math.toRadians(90))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(135))
                .lineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)), Math.toRadians(90))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -34, Math.toRadians(-90)), Math.toRadians(90))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-135))
                .lineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(10, -65, Math.toRadians(0)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(-45)), Math.toRadians(90))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(2, -52, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40, -65), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }
}