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

    public static List<Trajectory> getTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-41, -49, Math.toRadians(-45)))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-54, -56, Math.toRadians(-160)))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-160), SLOW_VEL_CONSTRAINT, SLOW_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-60, -59))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-32, -54, Math.toRadians(-90)))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-60, -52))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-60, -58))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -58))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-32, -60))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-90), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-50, -52, Math.toRadians(-45)))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(-55, -60))
                .build()
        );
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(-45), BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT)
                .lineToLinearHeading(new Pose2d(-68, -37, Math.toRadians(-90)))
                .build()
        );
        return trajectories;
    }
}