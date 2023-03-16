package team3647.frc2023.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;

public final class Trajectories {
    private static final PathConstraints fastConstraints = new PathConstraints(2.5, 2.5);

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);

    private static final PathConstraints slowConstraints = new PathConstraints(2, 2);

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

        public static final class ConeCubeBalanceBumpSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6.35, 0.93, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.80, 1.07, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathMidpoint =
                    new Pose2d(2.54, 2.79, FieldConstants.kZero);
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(3.93, 2.81, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(80)),
                                    fromPose(kThirdPathMidpoint, FieldConstants.kZero),
                                    fromPose(kThirdPathFinal, FieldConstants.kZero)));
        }

        public static final class ConeCubeBalanceFlatSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 3.85, FieldConstants.kZero);
            private static final Pose2d kFirstPathWaypoint1 =
                    new Pose2d(4.61, 4.73, FieldConstants.kZero);
            //     private static final Pose2d kFirstPathFinal = new Pose2d(6.90, 4.60, kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6.90, 4.60, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            //     private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39,
            // kZero);
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.80, 4.39, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial =
                    new Pose2d(1.80, 4.39, FieldConstants.kZero);
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(5.77, 3.86, FieldConstants.kZero);
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(3.5, 2.75, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kNinety),
                                    fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kFirstPathWaypoint1, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathFinal, Rotation2d.fromDegrees(-160))));
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(45)),
                                    fromPose(kThirdPathWaypoint1, Rotation2d.fromDegrees(-90)),
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

    public static final class Red {

        public static final class ConeCubeBumpSide {
            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBalanceBumpSide.kFirstTrajectory, Alliance.Red);
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBalanceBumpSide.kSecondTrajectory, Alliance.Red);
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBalanceBumpSide.kGoToBalance, Alliance.Red);
        }

        private Red() {}
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading) {
        return new PathPoint(pose.getTranslation(), heading, pose.getRotation());
    }

    private Trajectories() {}
}
