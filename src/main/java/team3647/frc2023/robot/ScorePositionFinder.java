package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import team3647.frc2023.util.SuperstructureState;

public class ScorePositionFinder {
    private final Supplier<Pose2d> robotPoseSupplier;
    private final List<ScoringPosition> possiblePositions;
    private final Map<Level, Map<GamePiece, SuperstructureState>>
            levelAndPieceToSuperstrucutreState;

    public ScorePositionFinder(
            Supplier<Pose2d> robotPoseSupplier,
            List<ScoringPosition> possiblePosition,
            Map<Level, Map<GamePiece, SuperstructureState>> levelAndPieceToSuperstrucutreState) {
        this.possiblePositions = possiblePosition;
        this.robotPoseSupplier = robotPoseSupplier;
        this.levelAndPieceToSuperstrucutreState = levelAndPieceToSuperstrucutreState;
    }

    public final SuperstructureState getSuperstructureState(Level wantedLevel) {
        GamePiece piece = getScoringPosition().piece;

        if (!this.levelAndPieceToSuperstrucutreState.containsKey(wantedLevel)) {
            return null; // stow position
        }

        var pieceToState = levelAndPieceToSuperstrucutreState.get(wantedLevel);

        if (!pieceToState.containsKey(piece)) {
            return null; // return cone level
        }

        return pieceToState.get(piece);
    }

    public final ScoringPosition getScoringPosition() {
        return getClosest(robotPoseSupplier.get(), this.possiblePositions);
    }

    public static class ScoringPosition {
        public ScoringPosition(Pose2d pose, GamePiece piece) {
            this.pose = pose;
            this.piece = piece;
        }

        public final Pose2d pose;
        public final GamePiece piece;
    }

    public static ScoringPosition getClosest(
            Pose2d drivePose, List<ScoringPosition> possiblePositions) {
        return Collections.min(
                possiblePositions,
                Comparator.comparing(
                                (ScoringPosition other) ->
                                        drivePose
                                                .getTranslation()
                                                .getDistance(other.pose.getTranslation()))
                        .thenComparing(
                                (ScoringPosition other) ->
                                        Math.abs(
                                                drivePose
                                                        .getRotation()
                                                        .minus(other.pose.getRotation())
                                                        .getRadians())));
    }

    public enum GamePiece {
        Cone,
        Cube
    }

    public enum Level {
        One,
        Two,
        Three,
        Stay
    }
}
