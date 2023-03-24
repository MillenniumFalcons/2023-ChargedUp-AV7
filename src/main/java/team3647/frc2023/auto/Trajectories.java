package team3647.frc2023.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;

public final class Trajectories {
    private static final PathConstraints fastConstraints = new PathConstraints(3, 2);

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    private static final PathConstraints defaultConstraintHigherAccel =
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 2.5);

    private static final PathConstraints slowConstraints = new PathConstraints(1.6, 1.9);

    public static final class Blue {
        public static final class Test {
            public static final Pose2d kStart = new Pose2d(0.0, 0.0, FieldConstants.kZero);
            public static final Pose2d kEnd =
                    new Pose2d(0, Units.inchesToMeters(-120), FieldConstants.kZero);
            public static final PathPlannerTrajectory kTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kStart, Rotation2d.fromDegrees(-90)),
                                    fromPose(kEnd, Rotation2d.fromDegrees(-90))));
        }

        public static final class ConeCubeCubeBalanceFlatSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 5.0, FieldConstants.kZero);
            //     private static final Pose2d kFirstPathFinal = new Pose2d(6.90, 4.60, kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6.70, 4.65, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            //     private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39,
            // kZero);
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(2.4, 4.39, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(5.9, 4.53, Rotation2d.fromDegrees(-45.0));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(6.78, 3.64, Rotation2d.fromDegrees(-45.0));
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(3.5, 2.75, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, Rotation2d.fromDegrees(-10)),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathFinal, Rotation2d.fromDegrees(-160))));
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(20)),
                                    fromPose(kThirdPathWaypoint1, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathFinal, FieldConstants.kOneEighty)));
        }

        public static final class ConeCubeBalanceBumpSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathWaypoint1 =
                    new Pose2d(3.67, 0.79, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6.9, 0.9, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathWaypoint1 = kFirstPathWaypoint1;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.80, 1.05, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(5.59, 0.96, FieldConstants.kZero);

            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathWaypoint1 =
                    new Pose2d(6.57, 1.66, Rotation2d.fromDegrees(45));
            private static final Pose2d kFourthPathFinal =
                    new Pose2d(7.17, 2.25, Rotation2d.fromDegrees(45));

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            slowConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));

            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            slowConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathWaypoint1, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));

            public static final PathPlannerTrajectory kGoToOutside =
                    PathPlanner.generatePath(
                            defaultConstraintHigherAccel,
                            List.of(
                                    fromPose(kThirdPathInitial, FieldConstants.kZero),
                                    fromPose(kThirdPathFinal, FieldConstants.kZero)));

            public static final PathPlannerTrajectory kIntakeCube =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(45)),
                                    fromPose(kFourthPathWaypoint1, Rotation2d.fromDegrees(45)),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(45))));
        }

        public static final class ConeCubeBalanceFlatSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 5.0, FieldConstants.kZero);
            //     private static final Pose2d kFirstPathFinal = new Pose2d(6.90, 4.60, kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6.70, 4.65, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            //     private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39,
            // kZero);
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(2.4, 4.39, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(5.9, 4.53, Rotation2d.fromDegrees(-45.0));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(6.78, 3.64, Rotation2d.fromDegrees(-45.0));
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(3.5, 2.75, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, Rotation2d.fromDegrees(-10)),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathFinal, Rotation2d.fromDegrees(-160))));
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(20)),
                                    fromPose(kThirdPathWaypoint1, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathFinal, FieldConstants.kOneEighty)));
        }

        public static final class ConeBalance {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 3.3, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(5.2, 3.3, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
        }

        private Blue() {}
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading) {
        return fromPose(pose, heading, -1);
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading, double velocityOverride) {
        return new PathPoint(pose.getTranslation(), heading, pose.getRotation(), velocityOverride);
    }

    private Trajectories() {}
}
