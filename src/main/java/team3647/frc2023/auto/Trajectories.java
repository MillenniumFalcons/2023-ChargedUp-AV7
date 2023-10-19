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
    private static final PathConstraints reallyFastConstraints = new PathConstraints(5, 3);
    private static final PathConstraints slowerConstraints = new PathConstraints(1.5, 1);

    private static final PathConstraints defaultConstraints =
            new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecSq);
    private static final PathConstraints defaultConstraintHigherAccel =
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 2.5);

    private static final PathConstraints defaultConstraintEvenHigherAccel =
            new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 3);

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

        public static final class coneCubeBalance {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.8, 3.3, FieldConstants.kZero);
            public static final Pose2d kFirstPathFinal = new Pose2d(5, 3.3, FieldConstants.kZero);
            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
        }

        public static final class ConeCubeCubeFlatSide {
            public static final Pose2d kFirstPathInitial =
                    ConeCubeBalanceFlatSide.kFirstPathInitial;

            private static final Pose2d kFirstPathFinal = ConeCubeBalanceFlatSide.kFirstPathFinal;

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal = ConeCubeBalanceFlatSide.kSecondPathFinal;

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            // avoid charging station
            private static final Pose2d kThirdPathWaypoint1 =
                    ConeCubeBalanceFlatSide.kThirdPathWaypoint1;
            // pick up cube 2
            private static final Pose2d kThirdPathWaypoint2 =
                    ConeCubeBalanceFlatSide.kThirdPathWaypoint2;
            // yeet cube point into the community
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(4.00, 5.03, Rotation2d.fromDegrees(25));

            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathFinal =
                    new Pose2d(6.76, 2.73, Rotation2d.fromDegrees(-60));

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
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            reallyFastConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(20)),
                                    fromPose(kThirdPathWaypoint1, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(-45.00)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(-165))));
            public static final PathPlannerTrajectory kGetLastCube =
                    PathPlanner.generatePath(
                            reallyFastConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, FieldConstants.kZero),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(-60))));
        }

        public static final class justDrive {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            public static final Pose2d kFirstPathFinal = new Pose2d(5.5, 0.5, FieldConstants.kZero);
            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
        }

        public static final class coneCubeCubeBumpSideNoBump {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(5.7, 0.98, FieldConstants.kZero);
            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.90, 1.1, FieldConstants.kZero);
            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(4.1, 0.6, Rotation2d.fromDegrees(30));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(5.80, 0.95, Rotation2d.fromDegrees(45));
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(6.25, 1.37, Rotation2d.fromDegrees(45));
            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            //     private static final Pose2d kFourthPathWaypoint1 = kThirdPathWaypoint2;
            //     private static final Pose2d kFourthPathWaypoint2 = kThirdPathWaypoint1;
            private static final Pose2d kFourthPathFinal = kThirdPathInitial;

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(-25)),
                                    fromPose(kThirdPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(25)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(45))));
            public static final PathPlannerTrajectory kFourthTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(-135)),
                                    fromPose(kThirdPathWaypoint1, FieldConstants.kOneEighty),
                                    //     fromPose(kFourthPathWaypoint1,
                                    // Rotation2d.fromDegrees(155)),
                                    //     fromPose(kFourthPathWaypoint2,
                                    // FieldConstants.kOneEighty),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(150))));
        }

        public static final class coneCubeCubeBalanceBumpSideNoBump {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(5.7, 0.98, FieldConstants.kZero);
            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.90, 1.1, FieldConstants.kZero);
            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(4.1, 0.6, Rotation2d.fromDegrees(30));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(5.80, 0.95, Rotation2d.fromDegrees(45));
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(6.25, 1.37, Rotation2d.fromDegrees(45));
            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathWaypoint1 =
                    new Pose2d(4.1, 0.6, FieldConstants.kZero);
            //     private static final Pose2d kFourthPathWaypoint1 = kThirdPathWaypoint2;
            //     private static final Pose2d kFourthPathWaypoint2 = kThirdPathWaypoint1;
            private static final Pose2d kFourthPathFinal =
                    new Pose2d(2.4, 2.09, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(-25)),
                                    fromPose(kThirdPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(25)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(45))));
            public static final PathPlannerTrajectory kFourthTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(-135)),
                                    fromPose(kFourthPathWaypoint1, FieldConstants.kOneEighty),
                                    //     fromPose(kFourthPathWaypoint1,
                                    // Rotation2d.fromDegrees(155)),
                                    //     fromPose(kFourthPathWaypoint2,
                                    // FieldConstants.kOneEighty),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(45))));
        }

        public static final class coneCubeCubeHoldBalanceBumpSideNoBump {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(5.7, 0.98, FieldConstants.kZero);
            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.90, 1.1, FieldConstants.kZero);
            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(4.1, 0.6, Rotation2d.fromDegrees(30));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(5.80, 0.95, Rotation2d.fromDegrees(45));
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(6.25, 1.37, Rotation2d.fromDegrees(45));
            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            //     private static final Pose2d kFourthPathWaypoint1 = kThirdPathWaypoint2;
            //     private static final Pose2d kFourthPathWaypoint2 = kThirdPathWaypoint1;
            private static final Pose2d kFourthPathFinal =
                    new Pose2d(5.3, 2.25, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));
            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(-25)),
                                    fromPose(kThirdPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(25)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(45))));
            public static final PathPlannerTrajectory kFourthTrajectory =
                    PathPlanner.generatePath(
                            slowConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(135)),
                                    //     fromPose(kFourthPathWaypoint1,
                                    // Rotation2d.fromDegrees(155)),
                                    //     fromPose(kFourthPathWaypoint2,
                                    // FieldConstants.kOneEighty),
                                    fromPose(kFourthPathFinal, FieldConstants.kOneEighty)));
        }

        public static final class ConeCubeBalanceBumpSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 0.5, FieldConstants.kZero);
            private static final Pose2d kFirstPathWaypoint1 =
                    new Pose2d(3.67, 0.79, FieldConstants.kZero);
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(6, 0.91 + 0.1, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            private static final Pose2d kSecondPathWaypoint1 = kFirstPathWaypoint1;
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(2.2 + 0.2, 1.05 + 0.03, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(4.9 + 0.2 + 0.15, 0.85, FieldConstants.kZero);

            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathWaypoint1 =
                    new Pose2d(6.37, 0.9, Rotation2d.fromDegrees(45));
            private static final Pose2d kFourthPathFinal =
                    new Pose2d(6.90, 1.43, Rotation2d.fromDegrees(45));

            private static final Pose2d kFifthPathInitial = kFourthPathFinal;
            private static final Pose2d kFifthPathWaypoint1 = kThirdPathFinal;
            private static final Pose2d kFifthPathWaypoint2 =
                    new Pose2d(3, 0.79, FieldConstants.kZero);
            private static final Pose2d kFifthPathFinal =
                    new Pose2d(2.2, 0.98 + 0.25 + 0.1, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            slowConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    // fromPose(kFirstPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));

            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraintHigherAccel,
                            List.of(
                                    fromPose(kSecondPathInitial, FieldConstants.kOneEighty),
                                    fromPose(kSecondPathWaypoint1, FieldConstants.kOneEighty, 1),
                                    fromPose(kSecondPathFinal, FieldConstants.kOneEighty)));

            public static final PathPlannerTrajectory kGoToOutsideAndIntake =
                    PathPlanner.generatePath(
                            defaultConstraintHigherAccel,
                            List.of(
                                    fromPose(kThirdPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathWaypoint1, FieldConstants.kZero, 1),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(10)),
                                    fromPose(kFourthPathWaypoint1, Rotation2d.fromDegrees(45)),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(45))));

            public static final PathPlannerTrajectory kIntakeCube =
                    PathPlanner.generatePath(
                            defaultConstraintEvenHigherAccel,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(45)),
                                    fromPose(kFourthPathWaypoint1, Rotation2d.fromDegrees(45)),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(45))));
            public static final PathPlannerTrajectory kShootFromBump =
                    PathPlanner.generatePath(
                            defaultConstraintEvenHigherAccel,
                            List.of(
                                    fromPose(kFifthPathInitial, Rotation2d.fromDegrees(-135)),
                                    fromPose(kFifthPathWaypoint1, FieldConstants.kOneEighty, 1),
                                    fromPose(kFifthPathWaypoint2, FieldConstants.kOneEighty),
                                    fromPose(kFifthPathFinal, FieldConstants.kZero)));
        }

        public static final class ConeCubeBalanceFlatSide {
            public static final Pose2d kFirstPathInitial =
                    ConeCubeCubeMidBalanceFlatSide.kFirstPathInitial;

            // Pick up cube 1
            private static final Pose2d kFirstPathFinal =
                    ConeCubeCubeMidBalanceFlatSide.kFirstPathFinal;

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            //     private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39,
            // kZero);
            private static final Pose2d kSecondPathFinal =
                    ConeCubeCubeMidBalanceFlatSide.kSecondPathFinal;

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;

            // avoid charging station
            private static final Pose2d kThirdPathWaypoint1 =
                    ConeCubeCubeMidBalanceFlatSide.kThirdPathWaypoint1;
            // pick up cube 2
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(6.82, 3.74, Rotation2d.fromDegrees(-45.0));
            // go on top of charging station
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(2.99, 2.75, FieldConstants.kZero);

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
                                    fromPose(kThirdPathFinal, FieldConstants.kOneEighty, 0.5)));
        }

        public static final class ConeCubeCubeMidBalanceFlatSide {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 5.0, FieldConstants.kZero);

            // Pick up cube 1
            private static final Pose2d kFirstPathFinal =
                    new Pose2d(5.7, 4.55, FieldConstants.kZero);

            private static final Pose2d kSecondPathInitial = kFirstPathFinal;
            //     private static final Pose2d kSecondPathFinal = new Pose2d(1.80, 4.39,
            // kZero);
            private static final Pose2d kSecondPathFinal =
                    new Pose2d(1.9, 4.35, FieldConstants.kZero);

            private static final Pose2d kThirdPathInitial = kSecondPathFinal;
            // avoid charging station
            private static final Pose2d kThirdPathWaypoint1 =
                    new Pose2d(4.1, 4.80, Rotation2d.fromDegrees(-30.0));
            private static final Pose2d kThirdPathWaypoint2 =
                    new Pose2d(5.95, 4.35, Rotation2d.fromDegrees(-45));
            // pick up cube 2
            private static final Pose2d kThirdPathFinal =
                    new Pose2d(6.4, 4, Rotation2d.fromDegrees(-45.0));

            private static final Pose2d kFourthPathInitial = kThirdPathFinal;
            private static final Pose2d kFourthPathWaypoint1 =
                    new Pose2d(4.03, 4.77, FieldConstants.kZero);

            private static final Pose2d kFourthPathFinal =
                    new Pose2d(1.9, 4.32, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathFinal, FieldConstants.kZero)));
            public static final PathPlannerTrajectory kSecondTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kSecondPathInitial, Rotation2d.fromDegrees(170)),
                                    fromPose(kSecondPathFinal, Rotation2d.fromDegrees(-160))));

            public static final PathPlannerTrajectory kThirdTrajectory =
                    PathPlanner.generatePath(
                            defaultConstraints,
                            List.of(
                                    fromPose(kThirdPathInitial, Rotation2d.fromDegrees(30)),
                                    fromPose(kThirdPathWaypoint1, FieldConstants.kZero),
                                    fromPose(kThirdPathWaypoint2, Rotation2d.fromDegrees(-30.00)),
                                    fromPose(kThirdPathFinal, Rotation2d.fromDegrees(-45.00))));

            public static final PathPlannerTrajectory kFourthTrajectory =
                    PathPlanner.generatePath(
                            fastConstraints,
                            List.of(
                                    fromPose(kFourthPathInitial, Rotation2d.fromDegrees(150)),
                                    fromPose(kFourthPathWaypoint1, FieldConstants.kOneEighty),
                                    fromPose(kFourthPathFinal, Rotation2d.fromDegrees(-150))));
        }

        public static final class ConeExitBalance {
            public static final Pose2d kFirstPathInitial =
                    new Pose2d(1.80, 3.3, FieldConstants.kZero);
            public static final Pose2d kFirstPathWaypoint1 =
                    new Pose2d(4.5, 3.3, FieldConstants.kZero);
            public static final Pose2d kFirstPathWaypoint2 =
                    new Pose2d(6.3, 3.3, FieldConstants.kZero);
            public static final Pose2d kFinalClimb = new Pose2d(4, 3.3, FieldConstants.kZero);

            public static final PathPlannerTrajectory kFirstTrajectory =
                    PathPlanner.generatePath(
                            slowerConstraints,
                            List.of(
                                    fromPose(kFirstPathInitial, FieldConstants.kZero),
                                    fromPose(kFirstPathWaypoint1, FieldConstants.kZero, 0.3),
                                    fromPose(kFirstPathWaypoint2, FieldConstants.kOneEighty, 0),
                                    fromPose(kFinalClimb, FieldConstants.kZero, -0.2)));
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
