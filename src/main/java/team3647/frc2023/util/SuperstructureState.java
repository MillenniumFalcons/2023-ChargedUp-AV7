package team3647.frc2023.util;

import java.util.Map;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.WristConstants;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.subsystems.Superstructure.StationType;

public class SuperstructureState {
    /** degrees */
    public final double armAngle;

    public final double wristAngle;

    public final double length;

    public final String name;

    // name parameter is just for debugging purposes

    private SuperstructureState(double armAngle, double length, double wristAngle, String name) {
        this.armAngle = armAngle;
        this.length = length;
        this.wristAngle = wristAngle;
        this.name = name;
    }

    private SuperstructureState(double armAngle, double length, String name) {
        this(armAngle, length, WristConstants.kHoldPosition, name);
    }

    public static final double kAdjustment = -4;
    public static final SuperstructureState stow =
            new SuperstructureState(
                    PivotConstants.kInitialAngle, ExtenderConstants.kMinimumPositionTicks, "stow");

    public static final SuperstructureState groundIntake =
            new SuperstructureState(
                    189.5, ExtenderConstants.kMinimumPositionTicks, "ground intake");

    public static final SuperstructureState groundIntakeReverse =
            new SuperstructureState(
                    180 - groundIntake.armAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "ground intake reverse");

    public static final SuperstructureState singleStation =
            new SuperstructureState(146, ExtenderConstants.kMinimumPositionTicks, "station");

    public static final SuperstructureState doubleStation =
            new SuperstructureState(
                    122.2,
                    ExtenderConstants.kDoubleStation,
                    WristConstants.kDoubleStationDegrees,
                    "double station");

    public static final SuperstructureState doubleStationReversed =
            new SuperstructureState(
                    180 - doubleStation.armAngle,
                    ExtenderConstants.kDoubleStation,
                    "double station reversed");

    public static final SuperstructureState coneOne =
            new SuperstructureState(
                    150 + kAdjustment,
                    ExtenderConstants.kMinimumPositionTicks,
                    WristConstants.kConeScoreAngle,
                    "cone low");
    public static final SuperstructureState coneTwo =
            new SuperstructureState(
                    145 + kAdjustment - 2,
                    ExtenderConstants.kLevelTwoExtendCone,
                    WristConstants.kConeScoreAngle,
                    "cone mid");
    public static final SuperstructureState coneThree =
            new SuperstructureState(
                    141 + kAdjustment,
                    ExtenderConstants.kLevelThreeExtendCone,
                    WristConstants.kConeScoreAngle,
                    "cone high");

    public static final SuperstructureState coneOneReversed =
            new SuperstructureState(
                    180 - coneOne.armAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "cone reversed low");
    public static final SuperstructureState coneTwoReversed =
            new SuperstructureState(
                    180 - coneTwo.armAngle,
                    ExtenderConstants.kLevelTwoExtendCone,
                    "cone reversed mid");
    public static final SuperstructureState coneThreeReversed =
            new SuperstructureState(
                    180 - coneThree.armAngle,
                    ExtenderConstants.kLevelThreeExtendCone - 4000,
                    "cone reversed high");

    public static final double kAngleAdjustment = 4.0;
    public static final SuperstructureState cubeZero =
            new SuperstructureState(
                    groundIntake.armAngle - 6 - kAngleAdjustment,
                    WristConstants.kCubeScoreAngle,
                    groundIntake.length,
                    "cube zero");
    public static final SuperstructureState cubeOne =
            new SuperstructureState(
                    125 - 5 - kAngleAdjustment,
                    WristConstants.kCubeScoreAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "cube low");
    public static final SuperstructureState cubeTwo =
            new SuperstructureState(
                    147.6 - 5 - kAngleAdjustment,
                    WristConstants.kCubeScoreAngle,
                    ExtenderConstants.kLevelTwoExtendCube,
                    "cube mid");
    public static final SuperstructureState cubeThree =
            new SuperstructureState(
                    145.3 - 5 - kAngleAdjustment,
                    WristConstants.kCubeScoreAngle,
                    ExtenderConstants.kLevelThreeExtendCube,
                    "cube high");

    public static final SuperstructureState cubeOneReversed =
            new SuperstructureState(
                    180 - cubeOne.armAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "cube reversed low");
    public static final SuperstructureState cubeTwoReversed =
            new SuperstructureState(
                    180 - cubeTwo.armAngle,
                    ExtenderConstants.kLevelTwoExtendCube,
                    "cube reversed mid");
    public static final SuperstructureState cubeThreeReversed =
            new SuperstructureState(
                    180 - cubeThree.armAngle,
                    ExtenderConstants.kLevelThreeExtendCube,
                    "cube reversed high");

    public static final SuperstructureState noLevel =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "no level");

    public static final Map<Level, Map<GamePiece, SuperstructureState>> kLevelPieceMap =
            Map.of(
                    Level.One,
                    Map.of(
                            GamePiece.Cone, coneOne,
                            GamePiece.Cube, cubeOne),
                    Level.Two,
                    Map.of(
                            GamePiece.Cone, coneTwo,
                            GamePiece.Cube, cubeTwo),
                    Level.Three,
                    Map.of(
                            GamePiece.Cone, coneThree,
                            GamePiece.Cube, cubeThree),
                    Level.Stay,
                    Map.of(
                            GamePiece.Cone, noLevel,
                            GamePiece.Cube, noLevel));
    public static final Map<StationType, SuperstructureState> kIntakePositionsMap =
            Map.of(
                    StationType.Double, doubleStation,
                    StationType.Single, singleStation,
                    StationType.Ground, groundIntake);
}
