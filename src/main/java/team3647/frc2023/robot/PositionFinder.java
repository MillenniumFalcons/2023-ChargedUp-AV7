package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.vision.AimingParameters;

public class PositionFinder {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<AimingParameters> getBestTarget;
    private final Supplier<Alliance> getColor;
    private final List<IntakePosition> possibleIntakePositions;
    private final Map<Level, Map<GamePiece, SuperstructureState>>
            levelAndPieceToSuperstrucutreState;
    private final List<ScoringPosition> kEmptyList = List.of();

    public PositionFinder(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<AimingParameters> getBestTarget,
            Supplier<Alliance> getColor,
            List<IntakePosition> possibleIntakePositions,
            Map<Level, Map<GamePiece, SuperstructureState>> levelAndPieceToSuperstrucutreState) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.possibleIntakePositions = possibleIntakePositions;
        this.getBestTarget = getBestTarget;
        this.getColor = getColor;
        this.levelAndPieceToSuperstrucutreState = levelAndPieceToSuperstrucutreState;
    }

    public final SuperstructureState getSuperstructureState(Level wantedLevel) {
        GamePiece piece = getClosestScoringPosition().piece;

        return getSuperstructureStateByPiece(wantedLevel, piece);
    }

    public final SuperstructureState getSuperstructureStateByPiece(
            Level wantedLevel, GamePiece piece) {
        if (wantedLevel == Level.Ground) {
            return SuperstructureState.cubeZero;
        }

        if (!this.levelAndPieceToSuperstrucutreState.containsKey(wantedLevel)) {
            return SuperstructureState.stow; // stow position
        }

        var pieceToState = levelAndPieceToSuperstrucutreState.get(wantedLevel);

        if (!pieceToState.containsKey(piece)) {
            return pieceToState.get(GamePiece.Cone); // return cone level
        }

        return pieceToState.get(piece);
    }

    private static Pose2d translatePose(Pose2d pose, Translation2d t, Rotation2d newRotation) {
        return new Pose2d(pose.getTranslation().plus(t), newRotation);
    }

    public final List<ScoringPosition> getScoringPositions() {
        var bestTarget = this.getBestTarget.get();
        var aprilTagPose = bestTarget.getFieldToGoal();

        if (bestTarget == AimingParameters.None) {
            return kEmptyList;
        }

        var cubePose =
                translatePose(
                        aprilTagPose,
                        FieldConstants.kBlueTransformTagCube.getTranslation(),
                        FieldConstants.kOneEighty);
        var conePoseLeft =
                translatePose(
                        aprilTagPose,
                        FieldConstants.kBlueTransformTagConeLeft.getTranslation(),
                        FieldConstants.kOneEighty);
        var conePoseRight =
                translatePose(
                        aprilTagPose,
                        FieldConstants.kBlueTransformTagConeRight.getTranslation(),
                        FieldConstants.kOneEighty);

        if (getColor.get() == Alliance.Red) {
            cubePose =
                    translatePose(
                            aprilTagPose,
                            FieldConstants.kRedTransformTagCube.getTranslation(),
                            FieldConstants.kZero);
            conePoseLeft =
                    translatePose(
                            aprilTagPose,
                            FieldConstants.kRedTransformTagConeLeft.getTranslation(),
                            FieldConstants.kZero);
            conePoseRight =
                    translatePose(
                            aprilTagPose,
                            FieldConstants.kRedTransformTagConeRight.getTranslation(),
                            FieldConstants.kZero);
        }

        return List.of(
                new ScoringPosition(conePoseLeft, GamePiece.Cone),
                new ScoringPosition(cubePose, GamePiece.Cube),
                new ScoringPosition(conePoseRight, GamePiece.Cone));
    }

    public final ScoringPosition getClosestScoringPosition() {
        return getClosestScoring(robotPoseSupplier.get(), this.getScoringPositions());
    }

    public final ScoringPosition getPositionBySide(Side side) {
        var scoringPositions = this.getScoringPositions();
        if (scoringPositions == kEmptyList) {
            return ScoringPosition.kNone;
        }

        return scoringPositions.get(side.listIndex);
    }

    public final IntakePosition getIntakePositionByStation(StationType station) {
        return getClosestIntake(
                getBestTarget.get().getFieldToGoal(),
                this.possibleIntakePositions.stream()
                        .filter(intakePos -> intakePos.station == station)
                        .toList());
    }

    public static class ScoringPosition implements HasPose {
        public static ScoringPosition kNone = new ScoringPosition(new Pose2d(), GamePiece.Cone);

        public ScoringPosition(Pose2d pose, GamePiece piece) {
            this.pose = pose;
            this.piece = piece;
        }

        @Override
        public Pose2d getPose() {
            return this.pose;
        }

        public final Pose2d pose;
        public final GamePiece piece;
    }

    public static class IntakePosition implements HasPose {
        public IntakePosition(Pose2d pose, StationType station) {
            this.pose = pose;
            this.station = station;
        }

        @Override
        public Pose2d getPose() {
            return this.pose;
        }

        public final StationType station;
        public final Pose2d pose;
    }

    public static interface HasPose {
        public Pose2d getPose();
    }

    // <T implements HasPose>
    public static ScoringPosition getClosestScoring(
            Pose2d drivePose, List<ScoringPosition> possiblePositions) {
        return Collections.min(
                possiblePositions,
                Comparator.comparing(
                                (ScoringPosition other) ->
                                        drivePose
                                                .getTranslation()
                                                .getDistance(other.getPose().getTranslation()))
                        .thenComparing(
                                (ScoringPosition other) ->
                                        Math.abs(
                                                drivePose
                                                        .getRotation()
                                                        .minus(other.getPose().getRotation())
                                                        .getRadians())));
    }

    public static IntakePosition getClosestIntake(
            Pose2d drivePose, List<IntakePosition> possiblePositions) {
        return Collections.min(
                possiblePositions,
                Comparator.comparing(
                                (IntakePosition other) ->
                                        drivePose
                                                .getTranslation()
                                                .getDistance(other.getPose().getTranslation()))
                        .thenComparing(
                                (IntakePosition other) ->
                                        Math.abs(
                                                drivePose
                                                        .getRotation()
                                                        .minus(other.getPose().getRotation())
                                                        .getRadians())));
    }

    public enum GamePiece {
        Cone,
        Cube
    }

    public enum Level {
        Ground,
        One,
        Two,
        Three,
        Stay
    }

    public enum Side {
        Left(0),
        Center(1),
        Right(2);

        final int listIndex;

        private Side(int listIndex) {
            this.listIndex = listIndex;
        }
    }
}
