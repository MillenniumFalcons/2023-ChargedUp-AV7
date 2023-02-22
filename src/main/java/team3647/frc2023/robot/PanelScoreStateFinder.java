package team3647.frc2023.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure.Level;
import team3647.lib.GroupPrinter;
import team3647.lib.PeriodicSubsystem;

public class PanelScoreStateFinder implements PeriodicSubsystem {
    private final GroupPrinter printer = GroupPrinter.getInstance();
    private final BooleanSupplier level1;
    private final BooleanSupplier level2;
    private final BooleanSupplier level3;

    private final BooleanSupplier column1;
    private final BooleanSupplier column2;
    private final BooleanSupplier column3;
    private final BooleanSupplier column4;
    private final BooleanSupplier column5;
    private final BooleanSupplier column6;
    private final BooleanSupplier column7;
    private final BooleanSupplier column8;
    private final BooleanSupplier column9;

    private Pose2d scorePose = new Pose2d();
    private PathPoint scorePoint;
    private Level scoreLevel = Level.coneTwo;
    private boolean[] levels = new boolean[3];
    private boolean[] positions = new boolean[9];
    private boolean hasCube = false;

    public PanelScoreStateFinder(
            BooleanSupplier level1,
            BooleanSupplier level2,
            BooleanSupplier level3,
            BooleanSupplier column1,
            BooleanSupplier column2,
            BooleanSupplier column3,
            BooleanSupplier column4,
            BooleanSupplier column5,
            BooleanSupplier column6,
            BooleanSupplier column7,
            BooleanSupplier column8,
            BooleanSupplier column9) {
        this.level1 = level1;
        this.level2 = level2;
        this.level3 = level3;
        this.column1 = column1;
        this.column2 = column2;
        this.column3 = column3;
        this.column4 = column4;
        this.column5 = column5;
        this.column6 = column6;
        this.column7 = column7;
        this.column8 = column8;
        this.column9 = column9;
    }

    @Override
    public void readPeriodicInputs() {
        levels[0] = level1.getAsBoolean();
        levels[1] = level2.getAsBoolean();
        levels[2] = level3.getAsBoolean();

        positions[0] = column1.getAsBoolean();
        positions[1] = column2.getAsBoolean();
        positions[2] = column3.getAsBoolean();
        positions[3] = column4.getAsBoolean();
        positions[4] = column5.getAsBoolean();
        positions[5] = column6.getAsBoolean();
        positions[6] = column7.getAsBoolean();
        positions[7] = column8.getAsBoolean();
        positions[8] = column9.getAsBoolean();

        scoreLevel = findScoreLevel();
        scorePoint = findScorePoint();
        scorePose = findScorePose();
        printer.addString("LEVEL NANEEE", () -> scoreLevel.name);
    }

    // positions [lvl1, lvl2, lvl3]
    private Level findScoreLevel() {
        if (levels[0] && levels[1] || levels[0] && levels[2] || levels[1] && levels[2]) {
            return Level.noLevel;
        }

        if (positions[1] || positions[4] || positions[7]) {
            hasCube = true;
            if (levels[0]) {
                return Level.cubeOne;
            } else if (levels[1]) {
                return Level.cubeTwo;
            } else if (levels[2]) {
                return Level.cubeThree;
            }
        } else {
            if (levels[0]) {
                return Level.coneOne;
            } else if (levels[1]) {
                return Level.coneTwo;
            } else if (levels[2]) {
                return Level.coneThree;
            }
        }

        return Level.noLevel;
    }

    private PathPoint findScorePoint() {
        // THIS DETERMINES DEFAULT POSE WHERE IT GOES IF ELLA DON'T PRESS ANYTHING
        int pressedIdx = 4;
        for (int i = 0; i < positions.length; i++) {
            if (positions[i]) {
                pressedIdx = i;
            }
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            Pose2d pose = FieldConstants.allBlueStations[pressedIdx];
            return new PathPoint(pose.getTranslation(), pose.getRotation());
        }
        Pose2d pose = FieldConstants.allRedStation[pressedIdx];
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }

    public Pose2d findScorePose() {
        int pressedIdx = 4;
        for (int i = 0; i < positions.length; i++) {
            if (positions[i]) {
                pressedIdx = i;
            }
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            Pose2d pose = FieldConstants.allBlueStations[pressedIdx];
            return pose;
        }
        Pose2d pose = FieldConstants.allRedStation[pressedIdx];
        return pose;
    }

    public Level getScoreLevel() {
        return scoreLevel;
    }

    public String getScoreLevelStr() {
        return scoreLevel.name;
    }

    public PathPoint getScorePoint() {
        return scorePoint;
    }

    public Pose2d getScorePose() {
        return scorePose;
    }

    public boolean getHasCube() {
        return hasCube;
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
