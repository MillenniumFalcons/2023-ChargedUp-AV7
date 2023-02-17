package team3647.frc2023.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure.Level;
import team3647.frc2023.util.Scoring;
import team3647.lib.GroupPrinter;
import team3647.lib.NetworkColorSensor.GamePiece;
import team3647.lib.PeriodicSubsystem;

public class ScoreStateFinder implements PeriodicSubsystem {
    private final Supplier<GamePiece> sensorGamePiece;
    private final Supplier<Pose2d> drivePose;
    private final BooleanSupplier goRightPosition;
    private final BooleanSupplier goTopLevel;
    private final BooleanSupplier goBottomLevel;
    private final GroupPrinter printer = GroupPrinter.getInstance();

    private PathPoint scorePoint;
    private Pose2d scorePose;
    private GamePiece heldPiece = GamePiece.NONE;

    private Level scoreLevel = Level.coneTwo;

    public ScoreStateFinder(
            Supplier<GamePiece> sensorGamePiece,
            Supplier<Pose2d> drivePose,
            BooleanSupplier goRightPosition,
            BooleanSupplier goTopLevel,
            BooleanSupplier goBottomLevel) {
        this.sensorGamePiece = sensorGamePiece;
        this.drivePose = drivePose;
        this.goRightPosition = goRightPosition;
        this.goTopLevel = goTopLevel;
        this.goBottomLevel = goBottomLevel;
    }

    @Override
    public void readPeriodicInputs() {
        heldPiece = sensorGamePiece.get();
        scorePoint = getScorePointFromSection(getUsedSections(), getScorePosition(heldPiece));
        scorePose = getScorePose2d(getUsedSections(), getScorePosition(heldPiece));
        scoreLevel = getScoreLevelFromPiece(heldPiece);
    }

    private List<Scoring.Section> getUsedSections() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return FieldConstants.allBlue;
        }
        return FieldConstants.allRed;
    }

    /**
     * @param usedSections which section (1,2,3 blue or red)
     * @param positionSupplier within section (left, right)
     * @return
     */
    private PathPoint getScorePointFromSection(
            List<Scoring.Section> sections, Scoring.Position position) {
        var driveT = drivePose.get().getTranslation();
        var tt = Scoring.getClosest(sections, driveT).getPose(position);
        return new PathPoint(tt.getTranslation(), tt.getRotation());
    }

    private Pose2d getScorePose2d(List<Scoring.Section> sections, Scoring.Position position) {
        var driveT = drivePose.get().getTranslation();
        Pose2d tt = Scoring.getClosest(sections, driveT).getPose(position);
        return tt;
    }

    private Scoring.Position getScorePosition(GamePiece heldPiece) {
        if (heldPiece == GamePiece.CUBE) {
            return Scoring.Position.MIDDLE;
        }

        if (this.goRightPosition.getAsBoolean()) {
            return Scoring.Position.RIGHT;
        }
        return Scoring.Position.LEFT;
    }

    private Level getScoreLevelFromPiece(GamePiece heldPiece) {
        if (heldPiece == GamePiece.NONE) {
            return Level.noLevel;
        }

        if (heldPiece == GamePiece.CONE) {
            if (goBottomLevel.getAsBoolean()) {
                return Level.coneOne;
            } else if (goTopLevel.getAsBoolean()) {
                return Level.coneThree;
            }
            return Level.coneTwo;
        }

        if (goBottomLevel.getAsBoolean()) {
            return Level.cubeOne;
        } else if (goTopLevel.getAsBoolean()) {
            return Level.cubeThree;
        }
        return Level.cubeTwo;
    }

    public Level getScoreLevel() {
        return scoreLevel;
    }

    public PathPoint getScorePoint() {
        return scorePoint;
    }

    public Pose2d getScorePose() {
        return scorePose;
    }

    @Override
    public String getName() {
        return "ScoreStateFinder";
    }
}
