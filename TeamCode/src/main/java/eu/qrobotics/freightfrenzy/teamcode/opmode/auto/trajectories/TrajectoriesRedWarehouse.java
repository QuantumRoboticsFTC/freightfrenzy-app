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
    public static Pose2d START_POSE = new Pose2d(8, -65, Math.toRadians(0));

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
                .lineToConstantHeading(new Vector2d(0, -39))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(1, -35, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(8, -76, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(46 + cycle * 4, -80), Math.toRadians(0))
                    .build()
            );

            trajectories.add(new TrajectoryBuilder(new Pose2d(38 + cycle * 2, -66, Math.toRadians(0)), Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -68))
                    .splineToConstantHeading(new Vector2d(6 + cycle, -70), Math.toRadians(90))
                    .lineToSplineHeading(new Pose2d(6 + cycle, -34 + cycle, Math.toRadians(-30)))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(8, -76, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, -80), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesB() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(12, -39))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(135))
                .lineToSplineHeading(new Pose2d(1, -35, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(7, -70, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(46 + cycle * 4, -72), Math.toRadians(0))
                    .build()
            );

            trajectories.add(new TrajectoryBuilder(new Pose2d(38 + cycle * 2, -66, Math.toRadians(0)), Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -70))
                    .splineToConstantHeading(new Vector2d(6 + cycle, -72), Math.toRadians(90))
                    .lineToSplineHeading(new Pose2d(6 + cycle, -34 + cycle, Math.toRadians(-30)))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(8, -76, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, -80), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesC() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(-90)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(1, -35, Math.toRadians(-45)))
                .build()
        );

        for (int cycle = 0; cycle < 3; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                    .lineToSplineHeading(new Pose2d(7, -70, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(46 + cycle * 4, -72), Math.toRadians(0))
                    .build()
            );

            trajectories.add(new TrajectoryBuilder(new Pose2d(38 + cycle * 2, -66, Math.toRadians(0)), Math.toRadians(180), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                    .lineToConstantHeading(new Vector2d(36 + cycle * 2, -70))
                    .splineToConstantHeading(new Vector2d(6 + cycle, -72), Math.toRadians(90))
                    .lineToSplineHeading(new Pose2d(6 + cycle, -34 + cycle, Math.toRadians(-30)))
                    .build()
            );
        }

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(8, -76, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, -80), Math.toRadians(0))
                .build()
        );

        return trajectories;
    }
}