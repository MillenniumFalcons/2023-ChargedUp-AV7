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

    public SuperstructureState addWristExtend(double degs, double extend) {
        return new SuperstructureState(
                this.armAngle, this.length + extend, this.wristAngle + degs, this.name);
    }

    public SuperstructureState noExtend() {
        return new SuperstructureState(
                this.armAngle, ExtenderConstants.kMinimumPositionTicks, this.wristAngle, this.name);
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
                    193, ExtenderConstants.kMinimumPositionTicks, 25.5, "ground intake cone");
    public static final SuperstructureState groundIntakeConeTipped =
            new SuperstructureState(-14.11, 1151.5, 11.89, "ground intake cone tipped");
    public static final SuperstructureState groundIntakeConeAuto =
            new SuperstructureState(
                    187.5, ExtenderConstants.kMinimumPositionTicks, 25, "ground intake cone");

    public static final SuperstructureState longTongueCube =
            new SuperstructureState(185, 47000, 34.2, "longTonguer");
    public static final SuperstructureState beforeLongTongueCube =
            new SuperstructureState(183, 0, -3, "longTonguer");
    public static final SuperstructureState groundIntakeCube =
            new SuperstructureState(
                    199, ExtenderConstants.kMinimumPositionTicks, 12, "ground intake cube");
    public static final SuperstructureState pushDownStation =
            new SuperstructureState(193, ExtenderConstants.kMinimumPositionTicks, 12, "push down");

    public static final SuperstructureState groundIntakeCubeLong =
            new SuperstructureState(184, 57500, 43, "ground intake long");

    public static final SuperstructureState doubleStationCone =
            new SuperstructureState(119, 18400, 103, "double station");
    public static final SuperstructureState doubleStationConeLying =
            new SuperstructureState(74, 3700, -58, "double station lying");
    public static final SuperstructureState doubleStationCube =
            new SuperstructureState(120, 16000, 117, "double station");

    public static final SuperstructureState coneOne =
            new SuperstructureState(
                    123.2, ExtenderConstants.kMinimumPositionTicks, 120, "cone low");
    public static final SuperstructureState coneTwo =
            new SuperstructureState(138, 16000, 92, "cone mid");
    public static final SuperstructureState coneThree =
            new SuperstructureState(138, 48750 + 800, 96.6, "cone high");

    public static final SuperstructureState coneThrow =
            new SuperstructureState(30, 16000, -17, "cone throw");

    public static final SuperstructureState coneThreeReversed =
            new SuperstructureState(41, 53000, 82, "cone high reversed");
    public static final SuperstructureState cubeTwoReversed =
            new SuperstructureState(40, 14136, -17.3, "cone mid reversed");
    public static final SuperstructureState cubeTwoReversedLoong =
            new SuperstructureState(38, 40000, -10, "cone mid reversed long");
    public static final SuperstructureState stowAfterConeThreeReversed =
            new SuperstructureState(90, 0, 112, "cone high reversed");
    public static final SuperstructureState cubeOne = coneOne;

    public static final SuperstructureState cubeTwo =
            new SuperstructureState(144, 14300, 63, "Cube two");
    public static final SuperstructureState cubeThree =
            new SuperstructureState(140, 46000, 39, "cube three");

    public static final SuperstructureState cubeThreeReversed =
            new SuperstructureState(42 + 1, 49950, -3, "cube high reversed");

    public static final SuperstructureState cubeOneReversedLong =
            new SuperstructureState(180, 0, 30.5, "cube high reversed");
    public static final SuperstructureState cubeShootReversedParallel =
            new SuperstructureState(25, 30000, "cube yeeter");

    public static final SuperstructureState untipReverse =
            new SuperstructureState(-5, ExtenderConstants.kMinimumPositionTicks, 30, "untip");

    public static final SuperstructureState backStow =
            new SuperstructureState(20, 0, 30, "stow back");

    public static final SuperstructureState lowCG = new SuperstructureState(160, 0, 104, "low cg");

    public static final SuperstructureState noLevel =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks * 2048,
                    "no level");

    public static final SuperstructureState stowScore =
            new SuperstructureState(
                    doubleStationCone.armAngle,
                    ExtenderConstants.kMinimumPositionTicks * 2048,
                    doubleStationCone.wristAngle,
                    "stow");

    public static final SuperstructureState stowIntake =
            new SuperstructureState(
                    doubleStationCone.armAngle,
                    ExtenderConstants.kMinimumPositionTicks * 2048,
                    10,
                    "stow");

    public static final SuperstructureState stowAll =
            new SuperstructureState(
                    PivotConstants.kInitialAngle,
                    ExtenderConstants.kMinimumPositionTicks * 2048,
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
                    StationType.Double, doubleStationCone,
                    StationType.Ground, doubleStationConeLying);
}
