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

    // ================================ practice bot values ==================================
    //     public static final SuperstructureState groundIntakeCone =
    //             new SuperstructureState(
    //                     189, ExtenderConstants.kMinimumPositionTicks, 62, "ground intake cone");

    //     public static final SuperstructureState groundIntakeCube =
    //             new SuperstructureState(
    //                     192.72, ExtenderConstants.kMinimumPositionTicks, 87.335, "ground intake
    // cube");

    //     public static final SuperstructureState doubleStation =
    //             new SuperstructureState(120, ExtenderConstants.kDoubleStation, 112, "double
    // station");

    //     public static final SuperstructureState coneOne =
    //             new SuperstructureState(180, ExtenderConstants.kMinimumPositionTicks, 108, "cone
    // low");
    //     public static final SuperstructureState coneTwo =
    //             new SuperstructureState(136.8, 13584, 111, "cone mid");
    //     public static final SuperstructureState coneThree =
    //             new SuperstructureState(139, 44500, 102.7, "cone high");

    //     public static final SuperstructureState coneThreeReversed =
    //             new SuperstructureState(31.727, 50673, 115.6, "cone high reversed");

    //     public static final SuperstructureState cubeOne =
    //             new SuperstructureState(
    //                     180, ExtenderConstants.kMinimumPositionTicks, 108, "cube  zero");

    //     public static final SuperstructureState cubeTwo =
    //             new SuperstructureState(142, 10500, 85, "cube mid");
    //     public static final SuperstructureState cubeThree =
    //             new SuperstructureState(142, 43372, 93.5, "cube high");

    //     public static final SuperstructureState cubeThreeReversed =
    //             new SuperstructureState(
    //                     coneThreeReversed.armAngle,
    //                     coneThreeReversed.length,
    //                     43.12,
    //                     "cube high reversed");

    //     public static final SuperstructureState noLevel =
    //             new SuperstructureState(
    //                     PivotConstants.kInitialAngle,
    //                     ExtenderConstants.kMinimumPositionTicks,
    //                     "no level");

    //     public static final SuperstructureState stowScore =
    //             new SuperstructureState(
    //                     doubleStation.armAngle,
    //                     ExtenderConstants.kMinimumPositionTicks,
    //                     doubleStation.wristAngle,
    //                     "stow");

    //     public static final SuperstructureState stowIntake =
    //             new SuperstructureState(
    //                     doubleStation.armAngle, ExtenderConstants.kMinimumPositionTicks, 30,
    // "stow");

    //     public static final SuperstructureState stowAll =
    //             new SuperstructureState(
    //                     PivotConstants.kInitialAngle,
    //                     ExtenderConstants.kMinimumPositionTicks,
    //                     30,
    //                     "stow");

    // ================================ comp bot values ==================================
    public static final SuperstructureState groundIntakeCone =
            new SuperstructureState(
                    187.5, ExtenderConstants.kMinimumPositionTicks, 35, "ground intake cone");
    public static final SuperstructureState groundIntakeConeAuto =
            new SuperstructureState(
                    187.5, ExtenderConstants.kMinimumPositionTicks, 30, "ground intake cone");

    public static final SuperstructureState groundIntakeCube =
            new SuperstructureState(
                    193, ExtenderConstants.kMinimumPositionTicks, 33, "ground intake cube");

    public static final SuperstructureState groundIntakeCubeLong =
            new SuperstructureState(184, 57500, 43, "ground intake long");

    public static final SuperstructureState doubleStation =
            new SuperstructureState(118 - 10 + 5, 12500 + 2048 + 500, 101, "double station");

    public static final SuperstructureState coneOne =
            new SuperstructureState(136, ExtenderConstants.kMinimumPositionTicks, 125, "cone low");
    public static final SuperstructureState coneTwo =
            new SuperstructureState(135, 17000, 93, "cone mid");
    public static final SuperstructureState coneThree =
            new SuperstructureState(139, 44500 + 2048 * 3, 82.5, "cone high");

    public static final SuperstructureState coneThreeReversed =
            new SuperstructureState(38, 51742, 108.31, "cone high reversed");
    public static final SuperstructureState stowAfterConeThreeReversed =
            new SuperstructureState(90, 0, 112, "cone high reversed");
    public static final SuperstructureState cubeOne = coneOne;

    public static final SuperstructureState cubeTwo = coneTwo;
    public static final SuperstructureState cubeThree = coneThree;

    public static final SuperstructureState cubeThreeReversed =
            new SuperstructureState(42, 49384, 7.3, "cube high reversed");

    public static final SuperstructureState cubeOneReversedLong =
            new SuperstructureState(180, 56000, 30.5, "cube high reversed");

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
                    doubleStation.armAngle, ExtenderConstants.kMinimumPositionTicks, 10, "stow");

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
                    StationType.Ground, groundIntakeCone);
}
