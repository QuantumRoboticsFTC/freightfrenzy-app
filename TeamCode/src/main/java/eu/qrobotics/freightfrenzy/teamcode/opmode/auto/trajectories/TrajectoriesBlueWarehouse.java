package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories;

import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.BASE_VEL_CONSTRAINT;
import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.NORMAL_ACCEL_CONSTRAINT;
import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.NORMAL_VEL_CONSTRAINT;
import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.SLOW_ACCEL_CONSTRAINT;
import static eu.qrobotics.freightfrenzy.teamcode.subsystems.DriveConstants.SLOW_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

public class TrajectoriesBlueWarehouse {
    public static Pose2d START_POSE = new Pose2d(8, 65, Math.toRadians(180));

    public static int CYCLE_COUNT = 7;

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

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-13, 65))
                .build()
        );

        for (int cycle = 0; cycle < CYCLE_COUNT; cycle++) {
            double xOffset = cycle * 1.25;
            double xDepositOffset = 2 * (cycle > 1 ? 1 : 0);
            double yOffset = 0.75 * cycle - (cycle > 4 ? 1 : 0);

            trajectories.add(makeTrajectoryBuilder(new Pose2d((cycle == 0 ? -10 : -14), 65, Math.toRadians(180)), Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(40 + xOffset, 68 - yOffset))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180), NORMAL_VEL_CONSTRAINT, NORMAL_ACCEL_CONSTRAINT)
                    .lineToSplineHeading(new Pose2d(40 + xOffset + 6, 68 - yOffset - 6, Math.toRadians(180-20)))
                    .build()
            );

            trajectories.add(makeTrajectoryBuilder(new Pose2d(40 + xOffset, 68, Math.toRadians(180)), Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-10, 68 + 0.3 * cycle + (cycle > 4 ? 1 : 0)))
                    .build()
            );
        }
        trajectories.add(makeTrajectoryBuilder(new Pose2d(-14, 66, Math.toRadians(180)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(44, 66))
                .build()
        );

        return trajectories;
    }
}