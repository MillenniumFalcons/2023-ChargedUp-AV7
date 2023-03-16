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

    public static final SuperstructureState groundIntake =
            new SuperstructureState(
                    193.6, ExtenderConstants.kMinimumPositionTicks, 62, "ground intake cone");

    //     public static final SuperstructureState groundIntakeCube =
    //             new SuperstructureState(
    //                     193.6, ExtenderConstants.kMinimumPositionTicks, 73, "ground intake
    // cube");

    public static final SuperstructureState doubleStation =
            new SuperstructureState(120, ExtenderConstants.kDoubleStation, 112, "double station");

    public static final SuperstructureState coneOne =
            new SuperstructureState(180, ExtenderConstants.kMinimumPositionTicks, 108, "cone low");
    public static final SuperstructureState coneTwo =
            new SuperstructureState(
                    137.4, ExtenderConstants.kLevelTwoExtendCone, 136.6, "cone mid");
    public static final SuperstructureState coneThree =
            new SuperstructureState(
                    137.4, ExtenderConstants.kLevelThreeExtendCone, 136.6, "cone high");

    // bade need to measure wrist
    public static final SuperstructureState coneThreeReversed =
            new SuperstructureState(33.58, 44300, 101.06, "cone high reversed");

    public static final SuperstructureState cubeZero =
            new SuperstructureState(180, ExtenderConstants.kMinimumPositionTicks, 108, "cube zero");

    public static final SuperstructureState cubeTwo =
            new SuperstructureState(146.5, 3300, 146.5, "cube mid");
    public static final SuperstructureState cubeThree =
            new SuperstructureState(146.5, 3300, 79, "cube high");

    // bade need to measure wrist
    public static final SuperstructureState cubeThreeReversed =
            new SuperstructureState(38.29, 43957, 20.8, "cube high reversed");

    public static final SuperstructureState noLevel =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    "no level");

    public static final SuperstructureState stowScore =
            new SuperstructureState(
                    doubleStation.armAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    doubleStation.wristAngle,
                    "stow");

    public static final SuperstructureState stowIntake =
            new SuperstructureState(
                    doubleStation.armAngle, ExtenderConstants.kMinimumPositionTicks, 30, "stow");

    public static final SuperstructureState stowAll =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks,
                    30,
                    "stow");

    public static final Map<Level, Map<GamePiece, SuperstructureState>> kLevelPieceMap =
            Map.of(
                    Level.One,
                    Map.of(
                            GamePiece.Cone, coneOne,
                            GamePiece.Cube, cubeZero),
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
                    StationType.Ground, groundIntake);
}
