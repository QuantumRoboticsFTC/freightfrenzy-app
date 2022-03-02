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
    public static Pose2d START_POSE = new Pose2d(-31, -65, Math.toRadians(-90));

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

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(135))
                .lineToSplineHeading(new Pose2d(-4, -51.5, Math.toRadians(-80)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                    .splineToSplineHeading(new Pose2d(16, -69, Math.toRadians(0)), Math.toRadians(0))
                    .lineToConstantHeading(new Vector2d(48 + cycle * 2, -69))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -67))
                    .splineToConstantHeading(new Vector2d(16, -67), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-4, -53, Math.toRadians(-80)), Math.toRadians(100))
                    .build()
            );
        }
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(16, -70, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(48, -70))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(135))
                .lineToSplineHeading(new Pose2d(-5, -52, Math.toRadians(-80)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                    .splineToSplineHeading(new Pose2d(16, -69, Math.toRadians(0)), Math.toRadians(0))
                    .lineToConstantHeading(new Vector2d(48 + cycle * 2, -69))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -67))
                    .splineToConstantHeading(new Vector2d(16, -67), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-4, -53, Math.toRadians(-80)), Math.toRadians(100))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(16, -70, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(48, -70))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(135))
                .lineToSplineHeading(new Pose2d(-5, -54, Math.toRadians(-80)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                    .splineToSplineHeading(new Pose2d(16, -69, Math.toRadians(0)), Math.toRadians(0))
                    .lineToConstantHeading(new Vector2d(48 + cycle * 2, -69))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -67))
                    .splineToConstantHeading(new Vector2d(16, -67), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-4, -53, Math.toRadians(-80)), Math.toRadians(100))
                    .build()
            );
        }
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(16, -70, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(48, -70))
                .build()
        );

        return trajectories;
    }
}