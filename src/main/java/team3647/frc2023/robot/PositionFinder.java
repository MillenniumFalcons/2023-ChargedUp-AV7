package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.vision.AimingParameters;

public class PositionFinder {
    private final Supplier<AimingParameters> getBestTarget;
    private final List<ScoringPosition> possibleScoringPositions;
    private final List<IntakePosition> possibleIntakePositions;
    private final Map<Level, Map<GamePiece, SuperstructureState>>
            levelAndPieceToSuperstrucutreState;

    public PositionFinder(
            Supplier<AimingParameters> getBestTarget,
            List<ScoringPosition> possibleScoringPositions,
            List<IntakePosition> possibleIntakePositions,
            Map<Level, Map<GamePiece, SuperstructureState>> levelAndPieceToSuperstrucutreState) {
        this.possibleScoringPositions = possibleScoringPositions;
        this.possibleIntakePositions = possibleIntakePositions;
        this.getBestTarget = getBestTarget;
        this.levelAndPieceToSuperstrucutreState = levelAndPieceToSuperstrucutreState;
    }

    public final SuperstructureState getSuperstructureState(Level wantedLevel) {
        GamePiece piece = getScoringPosition().piece;

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

    public final ScoringPosition getScoringPosition() {
        return getClosestScoring(
                getBestTarget.get().getFieldToGoal(), this.possibleScoringPositions);
    }

    public final IntakePosition getIntakePositionByStation(StationType station) {
        return getClosestIntake(
                getBestTarget.get().getFieldToGoal(),
                this.possibleIntakePositions.stream()
                        .filter(intakePos -> intakePos.station == station)
                        .toList());
    }

    public static class ScoringPosition implements HasPose {
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
}
