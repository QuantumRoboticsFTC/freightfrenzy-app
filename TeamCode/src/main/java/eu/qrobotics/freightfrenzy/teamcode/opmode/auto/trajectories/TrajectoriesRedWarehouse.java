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
    public static Pose2d START_POSE = new Pose2d(-31, -65, Math.toRadians(0));

    public static int CYCLE_COUNT = 1;

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

    private static TrajectoryBuilder makeTrajectoryBuilder(Pose2d pose, double startTangent) {
        return new TrajectoryBuilder(pose, startTangent, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-10, -65))
                .build()
        );

        for (int cycle = 0; cycle < CYCLE_COUNT; cycle++) {
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                    .lineToConstantHeading(new Vector2d(46 + cycle * 4, -67))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(new Pose2d(48 + cycle * 4, -65, Math.toRadians(0)), Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-10, -66))
                    .build()
            );
        }
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(52, -67))
                .build()
        );

        return trajectories;
    }
}