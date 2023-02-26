package team3647.frc2023.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import team3647.frc2023.constants.AutoConstants;

public final class Trajectories {

    private static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);
    private static final Rotation2d kZero = new Rotation2d();

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final class Blue {
        private static final Pose2d kRightFirstPathInitial = new Pose2d(1.8, 0.45, kZero);
        private static final Pose2d kRightFirstPathFinal = new Pose2d(6.35, 0.92, kZero);
        private static final Pose2d kRightSecondPathInitial = kRightFirstPathFinal;
        private static final Pose2d kRightSecondPathFinal = new Pose2d(1.80, 1.05, kZero);
        private static final Pose2d kRightThirdPathInitial = kRightSecondPathFinal;
        private static final Pose2d kRightThirdPathWaypoint1 = new Pose2d(2.39, 2.69, kZero);
        private static final Pose2d kRightThirdPathFinal = new Pose2d(4, 2.75, kZero);
        public static final PathPlannerTrajectory rightSideConeCubeFirst =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightFirstPathInitial, kZero),
                                fromPose(kRightFirstPathFinal, kZero)));
        public static final PathPlannerTrajectory rightSideConeCubeSecond =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightSecondPathInitial, kZero),
                                fromPose(kRightSecondPathFinal, kOneEighty)));
        public static final PathPlannerTrajectory rightGoToBalance =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightThirdPathInitial, Rotation2d.fromDegrees(95)),
                                fromPose(kRightThirdPathWaypoint1, kZero),
                                fromPose(kRightThirdPathFinal, kZero)));
        public static final PathPlannerTrajectory leftSideConeCubeFirst = null;
        public static final PathPlannerTrajectory leftSideConeCubeSecond = null;
        public static final PathPlannerTrajectory leftGoToBalance = null;

        private Blue() {}
    }

    public static final class Red {

        public static final PathPlannerTrajectory rightSideConeCubeFirst = null;
        public static final PathPlannerTrajectory rightSideConeCubeSecond = null;
        public static final PathPlannerTrajectory rightGoToBalance = null;
        public static final PathPlannerTrajectory leftSideConeCubeFirst = null;
        public static final PathPlannerTrajectory leftSideConeCubeSecond = null;
        public static final PathPlannerTrajectory leftGoToBalance = null;

        private Red() {}
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading) {
        return new PathPoint(pose.getTranslation(), heading, pose.getRotation());
    }

    private Trajectories() {}
}
