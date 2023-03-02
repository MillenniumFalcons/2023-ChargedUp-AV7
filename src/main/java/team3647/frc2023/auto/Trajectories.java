package team3647.frc2023.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;

public final class Trajectories {

    private static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);
    private static final Rotation2d kZero = new Rotation2d();

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final class Blue {
        private static final Pose2d kRightFirstPathInitial = new Pose2d(1.8, 0.45, kZero);
        private static final Pose2d kRightFirstPathFinal = new Pose2d(6.7, 0.92, kZero);
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
                                fromPose(kRightSecondPathInitial, kOneEighty),
                                fromPose(kRightSecondPathFinal, kOneEighty)));
        public static final PathPlannerTrajectory rightGoToBalance =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightThirdPathInitial, Rotation2d.fromDegrees(95)),
                                fromPose(kRightThirdPathWaypoint1, kZero),
                                fromPose(kRightThirdPathFinal, kZero)));

        private static final Pose2d kLeftFirstPathInitial = new Pose2d(1.8, 4.95, kZero);
        private static final Pose2d kLeftFirstPathFinal = new Pose2d(6.7, 4.60, kZero);
        private static final Pose2d kLeftSecondPathInitial = kLeftFirstPathFinal;
        private static final Pose2d kLeftSecondPathFinal = new Pose2d(1.80, 4.40, kZero);
        private static final Pose2d kLeftThirdPathInitial =
                new Pose2d(1.80, 4.40, Rotation2d.fromDegrees(0));
        private static final Pose2d kLeftThirdPathWayFinal =
                new Pose2d(6.60, 4.00, Rotation2d.fromDegrees(-45));
        private static final Pose2d kLeftFourthPathInitial =
                new Pose2d(6.60, 4.0, Rotation2d.fromDegrees(-45));
        private static final Pose2d kLeftFourthPathFinal =
                new Pose2d(1.8, 3.85, Rotation2d.fromDegrees(0));

        public static final PathPlannerTrajectory leftSideConeCubeConeFirst =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kLeftFirstPathInitial, kZero),
                                fromPose(kLeftFirstPathFinal, kZero)));
        ;
        public static final PathPlannerTrajectory leftSideConeCubeConeSecond =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kLeftSecondPathInitial, kZero),
                                fromPose(kLeftSecondPathFinal, kZero)));
        ;
        public static final PathPlannerTrajectory leftSideConeCubeConeThird =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kLeftThirdPathInitial, Rotation2d.fromDegrees(45)),
                                fromPose(kLeftThirdPathWayFinal, Rotation2d.fromDegrees(-45))));
        ;
        public static final PathPlannerTrajectory leftSideConeCubeConeFourth =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kLeftFourthPathInitial, Rotation2d.fromDegrees(120)),
                                fromPose(kLeftFourthPathFinal, Rotation2d.fromDegrees(-90))));

        // cone cone balance
        private static final Pose2d kRightConeConeFirstPathInitial = new Pose2d(1.8, 0.45, kZero);
        private static final Pose2d kRightConeConeFirstPathFinal = new Pose2d(6.7, 0.92, kZero);
        private static final Pose2d kRightConeConeSecondPathInitial = kRightConeConeFirstPathFinal;
        private static final Pose2d kRightConeConeSecondPathFinal = new Pose2d(3.88, 2.15, kZero);

        public static final PathPlannerTrajectory rightSideConeConeFirst =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightConeConeFirstPathInitial, kZero),
                                fromPose(kRightConeConeFirstPathFinal, kZero)));
        public static final PathPlannerTrajectory rightSideConeConeSecond =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(
                                        kRightConeConeSecondPathInitial,
                                        Rotation2d.fromDegrees(-110)),
                                fromPose(
                                        kRightConeConeSecondPathFinal,
                                        Rotation2d.fromDegrees(3.21))));

        private Blue() {}
    }

    public static final class Red {
        private static final Pose2d kRightFirstPathInitial =
                FieldConstants.flipBluePose(Blue.kRightFirstPathInitial);
        private static final Pose2d kRightFirstPathFinal =
                FieldConstants.flipBluePose(Blue.kRightFirstPathFinal);
        private static final Pose2d kRightSecondPathInitial =
                FieldConstants.flipBluePose(Blue.kRightSecondPathInitial);
        private static final Pose2d kRightSecondPathFinal =
                FieldConstants.flipBluePose(Blue.kRightSecondPathFinal);
        private static final Pose2d kRightThirdPathInitial =
                FieldConstants.flipBluePose(Blue.kRightThirdPathInitial);
        private static final Pose2d kRightThirdPathWaypoint1 =
                FieldConstants.flipBluePose(Blue.kRightThirdPathWaypoint1);
        private static final Pose2d kRightThirdPathFinal =
                FieldConstants.flipBluePose(Blue.kRightThirdPathFinal);

        public static final PathPlannerTrajectory rightSideConeCubeFirst = null;
        public static final PathPlannerTrajectory rightSideConeCubeSecond = null;
        public static final PathPlannerTrajectory rightGoToBalance = null;

        public static final PathPlannerTrajectory leftSideConeCubeFirst =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightFirstPathInitial, kOneEighty),
                                fromPose(kRightFirstPathFinal, kOneEighty)));
        public static final PathPlannerTrajectory leftSideConeCubeSecond =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightSecondPathInitial, kZero),
                                fromPose(kRightSecondPathFinal, kZero)));
        public static final PathPlannerTrajectory leftGoToBalance =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightThirdPathInitial, Rotation2d.fromDegrees(95)),
                                fromPose(kRightThirdPathWaypoint1, kOneEighty),
                                fromPose(kRightThirdPathFinal, kOneEighty)));

        // cone cone balance
        private static final Pose2d kRightConeConeFirstPathInitial =
                FieldConstants.flipBluePose(Blue.kRightConeConeFirstPathInitial);
        private static final Pose2d kRightConeConeFirstPathFinal =
                FieldConstants.flipBluePose(Blue.kRightConeConeFirstPathFinal);
        private static final Pose2d kRightConeConeSecondPathInitial =
                FieldConstants.flipBluePose(Blue.kRightConeConeSecondPathInitial);
        private static final Pose2d kRightConeConeSecondPathFinal =
                FieldConstants.flipBluePose(Blue.kRightConeConeSecondPathFinal);
        ;

        public static final PathPlannerTrajectory rightSideConeConeFirst =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(kRightConeConeFirstPathInitial, kOneEighty),
                                fromPose(kRightConeConeFirstPathFinal, kOneEighty)));
        public static final PathPlannerTrajectory rightSideConeConeSecond =
                PathPlanner.generatePath(
                        defaultConstraints,
                        List.of(
                                fromPose(
                                        kRightConeConeSecondPathInitial,
                                        Rotation2d.fromDegrees(-110.00).rotateBy(kOneEighty)),
                                fromPose(
                                        kRightConeConeSecondPathFinal,
                                        Rotation2d.fromDegrees(3.21).rotateBy(kOneEighty))));

        private Red() {}
    }

    public static PathPoint fromPose(Pose2d pose, Rotation2d heading) {
        return new PathPoint(pose.getTranslation(), heading, pose.getRotation());
    }

    private Trajectories() {}
}
