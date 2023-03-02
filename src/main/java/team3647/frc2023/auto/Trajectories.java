package team3647.frc2023.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import team3647.frc2023.constants.AutoConstants;

public final class Trajectories {

    private static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);
    private static final Rotation2d kZero = new Rotation2d();
    private static final Rotation2d kNinety = Rotation2d.fromDegrees(90);

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final class Blue {
        public static final class ConeCubeBumpSide {
            private static final Pose2d kFirstPathInitial = new Pose2d(1.80, 0.5, kZero);
            private static final Pose2d kFirstPathFinal = new Pose2d(6.35, 0.93, kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 1.07, kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathMidpoint = new Pose2d(2.54, 2.79, kZero);
            private static final Pose2d kThirdPathFinal = new Pose2d(3.93, 2.81, kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, kZero),
                                    fromPose(kFirstPathFinal, kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, kOneEighty),
                                    fromPose(kSecondPathFinal, kOneEighty)));
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(80)),
                                    fromPose(kThirdPathMidpoint, kZero),
                                    fromPose(kThirdPathFinal, kZero)));
        }

        public static final class ConeCubeConeFlat {
            private static final Pose2d kFirstPathInitial = new Pose2d(1.80, 3.85, kZero);
            private static final Pose2d kFirstPathFinal = new Pose2d(6.41, 4.64, kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39, kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(6.74, 3.96, Rotation2d.fromDegrees(-60));

            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathFinal = new Pose2d(1.8, 4.95, kOneEighty);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, kNinety),
                                    fromPose(kFirstPathFinal, kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, kOneEighty),
                                    fromPose(kSecondPathFinal, Rotation2d.fromDegrees(-160))));
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(50)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(-50))));
            public static final PathPlannerTrajectory kFourthTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(140)),
                                    fromPose(kFourthPathFinal, kOneEighty)));
        }

        private Blue() {}
    }

    public static final class Red {

        public static final class ConeCubeBumpSide {
            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBumpSide.kFirstTrajectory, Alliance.Red);
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBumpSide.kSecondTrajectory, Alliance.Red);
            public static final PathPlannerTrajectory kGoToBalance =
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                            Blue.ConeCubeBumpSide.kGoToBalance, Alliance.Red);
        }

        private Red() {}
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading) {
        return new PathPoint(pose.getTranslation(), heading, pose.getRotation());
    }

    private Trajectories() {}
}
