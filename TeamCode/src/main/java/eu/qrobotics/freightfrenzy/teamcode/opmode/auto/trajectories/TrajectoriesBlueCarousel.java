package eu.qrobotics.freightfrenzy.teamcode.opmode.auto.trajectories;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

public class TrajectoriesBlueCarousel {
    public static Pose2d START_POSE = new Pose2d(-40, 65, Math.toRadians(180));

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

    public static List<Trajectory> getTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(220))
                .splineToSplineHeading(new Pose2d(-60, 24, Math.toRadians(270)), Math.toRadians(270))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-53, 24))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-60, 60, Math.toRadians(330)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-64, 36, Math.toRadians(270)))
                .build()
        );

        return trajectories;
    }
}