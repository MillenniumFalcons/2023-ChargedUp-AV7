// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.IntakePosition;
import team3647.frc2023.robot.PositionFinder.ScoringPosition;
import team3647.frc2023.subsystems.Superstructure.StationType;

/** Add your docs here. */
public class FieldConstants {
    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);

    private static final Rotation2d kBlueScoreRotation = Rotation2d.fromDegrees(180);
    private static final Rotation2d kRedRedRotation = Rotation2d.fromDegrees(0);

    private static final Rotation2d kBlueDoubleSubstationRotation = Rotation2d.fromDegrees(0);
    private static final Rotation2d kRedDoubleSubstationRotation = Rotation2d.fromDegrees(180);

    private static final Rotation2d kBlueSingleStationRotation = Rotation2d.fromDegrees(90);
    private static final Rotation2d kRedSingleStationRotation = Rotation2d.fromDegrees(-90);

    private static final double kBlueXm = 1.80 + Units.inchesToMeters(3); // bumpers 3inches thick
    private static final double kBlueNineYm = 0.52;
    private static final double kBlueEightYm = 1.05;
    private static final double kBlueSevenYm = 1.64;
    private static final double kBlueSixYm = 2.19;
    private static final double kBlueFiveYm = 2.75;
    private static final double kBlueFourYm = 3.30;
    private static final double kBlueThreeYm = 3.87;
    private static final double kBlueTwoYm = 4.43;
    private static final double kBlueOneYm = 4.97;

    private static final double kRedXm = 14.73 - Units.inchesToMeters(3);
    private static final double kRedOneYm = kBlueNineYm;
    private static final double kRedTwoYm = kBlueEightYm;
    private static final double kRedThreeYm = kBlueSevenYm;
    private static final double kRedFourYm = kBlueSixYm;
    private static final double kRedFiveYm = kBlueFiveYm;
    private static final double kRedSixYm = kBlueFourYm;
    private static final double kRedSevenYm = kBlueThreeYm;
    private static final double kRedEightYm = kBlueTwoYm;
    private static final double kRedNineYm = kBlueOneYm;

    private static final double kBlueDoubleSubstationLeftYm = 7.35;
    private static final double kBlueDoubleSubstationRightYm = 6;
    private static final double kBlueDoubleSubstationXm = 15.66;

    private static final double kBlueSingleStationXm = 0.0;
    private static final double kBlueSingleStationYm = 0.0;
    private static final double kRedSingleStationXm = 0.0;
    private static final double kRedSingleStationYm = kBlueSingleStationYm;

    private static final double kRedDoubleSubstationLeftYm = kBlueDoubleSubstationRightYm;
    private static final double kRedDoubleSubstationRightYm = kBlueDoubleSubstationLeftYm;
    private static final double kRedDoubleSubstationXm = 0.8;

    private static final Pose2d kBlueOne = new Pose2d(kBlueXm, kBlueOneYm, kBlueScoreRotation);
    private static final Pose2d kBlueTwo = new Pose2d(kBlueXm, kBlueTwoYm, kBlueScoreRotation);
    private static final Pose2d kBlueThree = new Pose2d(kBlueXm, kBlueThreeYm, kBlueScoreRotation);
    private static final Pose2d kBlueFour = new Pose2d(kBlueXm, kBlueFourYm, kBlueScoreRotation);
    private static final Pose2d kBlueFive = new Pose2d(kBlueXm, kBlueFiveYm, kBlueScoreRotation);
    private static final Pose2d kBlueSix = new Pose2d(kBlueXm, kBlueSixYm, kBlueScoreRotation);
    private static final Pose2d kBlueSeven = new Pose2d(kBlueXm, kBlueSevenYm, kBlueScoreRotation);
    private static final Pose2d kBlueEight = new Pose2d(kBlueXm, kBlueEightYm, kBlueScoreRotation);
    private static final Pose2d kBlueNine = new Pose2d(kBlueXm, kBlueNineYm, kBlueScoreRotation);

    private static final Pose2d kRedOne = new Pose2d(kRedXm, kRedOneYm, kRedRedRotation);
    private static final Pose2d kRedTwo = new Pose2d(kRedXm, kRedTwoYm, kRedRedRotation);
    private static final Pose2d kRedThree = new Pose2d(kRedXm, kRedThreeYm, kRedRedRotation);
    private static final Pose2d kRedFour = new Pose2d(kRedXm, kRedFourYm, kRedRedRotation);
    private static final Pose2d kRedFive = new Pose2d(kRedXm, kRedFiveYm, kRedRedRotation);
    private static final Pose2d kRedSix = new Pose2d(kRedXm, kRedSixYm, kRedRedRotation);
    private static final Pose2d kRedSeven = new Pose2d(kRedXm, kRedSevenYm, kRedRedRotation);
    private static final Pose2d kRedEight = new Pose2d(kRedXm, kRedEightYm, kRedRedRotation);
    private static final Pose2d kRedNine = new Pose2d(kRedXm, kRedNineYm, kRedRedRotation);

    private static final Pose2d kBlueDoubleSubstationLeft =
            new Pose2d(
                    kBlueDoubleSubstationXm,
                    kBlueDoubleSubstationLeftYm,
                    kBlueDoubleSubstationRotation);
    private static final Pose2d kBlueDoubleSubstationRight =
            new Pose2d(
                    kBlueDoubleSubstationXm,
                    kBlueDoubleSubstationRightYm,
                    kBlueDoubleSubstationRotation);
    private static final Pose2d kBlueSingleStation =
            new Pose2d(kBlueSingleStationXm, kBlueSingleStationYm, kBlueSingleStationRotation);

    private static final Pose2d kRedDoubleSubstationLeft =
            new Pose2d(
                    kRedDoubleSubstationXm,
                    kRedDoubleSubstationLeftYm,
                    kRedDoubleSubstationRotation);
    private static final Pose2d kRedDoubleSubstationRight =
            new Pose2d(
                    kRedDoubleSubstationXm,
                    kRedDoubleSubstationRightYm,
                    kRedDoubleSubstationRotation);
    private static final Pose2d kRedSingleStation =
            new Pose2d(kRedSingleStationXm, kRedSingleStationYm, kRedSingleStationRotation);

    public static final Pose2d kGroundIntake = new Pose2d();

    public static final List<ScoringPosition> kScoringPositions =
            List.of(
                    new ScoringPosition(kBlueOne, GamePiece.Cone),
                    new ScoringPosition(kBlueTwo, GamePiece.Cube),
                    new ScoringPosition(kBlueThree, GamePiece.Cone),
                    new ScoringPosition(kBlueFour, GamePiece.Cone),
                    new ScoringPosition(kBlueFive, GamePiece.Cube),
                    new ScoringPosition(kBlueSix, GamePiece.Cone),
                    new ScoringPosition(kBlueSeven, GamePiece.Cone),
                    new ScoringPosition(kBlueEight, GamePiece.Cube),
                    new ScoringPosition(kBlueNine, GamePiece.Cone),
                    new ScoringPosition(kRedOne, GamePiece.Cone),
                    new ScoringPosition(kRedTwo, GamePiece.Cube),
                    new ScoringPosition(kRedThree, GamePiece.Cone),
                    new ScoringPosition(kRedFour, GamePiece.Cone),
                    new ScoringPosition(kRedFive, GamePiece.Cube),
                    new ScoringPosition(kRedSix, GamePiece.Cone),
                    new ScoringPosition(kRedSeven, GamePiece.Cone),
                    new ScoringPosition(kRedEight, GamePiece.Cube),
                    new ScoringPosition(kRedNine, GamePiece.Cone));

    public static final List<IntakePosition> kIntakePositions =
            List.of(
                    new IntakePosition(kBlueDoubleSubstationLeft, StationType.Double),
                    new IntakePosition(kBlueDoubleSubstationRight, StationType.Double),
                    new IntakePosition(kRedDoubleSubstationLeft, StationType.Double),
                    new IntakePosition(kRedDoubleSubstationRight, StationType.Double),
                    new IntakePosition(kBlueSingleStation, StationType.Single),
                    new IntakePosition(kRedSingleStation, StationType.Single),
                    new IntakePosition(kGroundIntake, StationType.Ground));

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
                FieldConstants.kFieldLength - translation.getX(), translation.getY());
    }

    private FieldConstants() {}
}
