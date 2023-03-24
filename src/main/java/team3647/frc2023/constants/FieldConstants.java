// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FieldConstants {
    public static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);
    public static final Rotation2d kZero = new Rotation2d();
    public static final Rotation2d kNinety = Rotation2d.fromDegrees(90);

    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);

    private static final Rotation2d kBlueScoreRotation = FieldConstants.kOneEighty;
    private static final Rotation2d kRedRedRotation = FieldConstants.kZero;

    private static final Rotation2d kBlueDoubleSubstationRotation = FieldConstants.kZero;
    private static final Rotation2d kRedDoubleSubstationRotation = FieldConstants.kOneEighty;

    private static final Rotation2d kBlueSingleStationRotation = FieldConstants.kNinety;
    private static final Rotation2d kRedSingleStationRotation = FieldConstants.kNinety;

    public static final double kXAdjustment = Units.inchesToMeters(10);

    public static final double kActualBlueXm = 1.80;
    public static final double kBlueXm = kActualBlueXm - kXAdjustment; // bumpers 3inches thick
    public static final double kBlueNineYm = 0.52;
    public static final double kBlueEightYm = 1.05;
    public static final double kBlueSevenYm = 1.64;
    public static final double kBlueSixYm = 2.19;
    public static final double kBlueFiveYm = 2.75;
    public static final double kBlueFourYm = 3.30;
    public static final double kBlueThreeYm = 3.87;
    public static final double kBlueTwoYm = 4.43;
    public static final double kBlueOneYm = 4.97;

    public static final double kActualRedXm = 14.73;
    public static final double kRedXm = kActualRedXm + kXAdjustment;
    public static final double kRedOneYm = kBlueNineYm;
    public static final double kRedTwoYm = kBlueEightYm;
    public static final double kRedThreeYm = kBlueSevenYm;
    public static final double kRedFourYm = kBlueSixYm;
    public static final double kRedFiveYm = kBlueFiveYm;
    public static final double kRedSixYm = kBlueFourYm;
    public static final double kRedSevenYm = kBlueThreeYm;
    public static final double kRedEightYm = kBlueTwoYm;
    public static final double kRedNineYm = kBlueOneYm;

    private static final double kBlueDoubleSubstationLeftYm = 7.35;
    private static final double kBlueDoubleSubstationRightYm = 6;
    private static final double kBlueDoubleSubstationXm = 15.66;

    private static final double kBlueSingleStationYm = 0.0;

    public static final Pose2d kOrigin = new Pose2d();

    public static final Pose2d kBlueOne = new Pose2d(kBlueXm, kBlueOneYm, kBlueScoreRotation);
    public static final Pose2d kBlueTwo = new Pose2d(kBlueXm, kBlueTwoYm, kBlueScoreRotation);
    public static final Pose2d kBlueThree = new Pose2d(kBlueXm, kBlueThreeYm, kBlueScoreRotation);
    public static final Pose2d kBlueFour = new Pose2d(kBlueXm, kBlueFourYm, kBlueScoreRotation);
    public static final Pose2d kBlueFive = new Pose2d(kBlueXm, kBlueFiveYm, kBlueScoreRotation);
    public static final Pose2d kBlueSix = new Pose2d(kBlueXm, kBlueSixYm, kBlueScoreRotation);
    public static final Pose2d kBlueSeven = new Pose2d(kBlueXm, kBlueSevenYm, kBlueScoreRotation);
    public static final Pose2d kBlueEight = new Pose2d(kBlueXm, kBlueEightYm, kBlueScoreRotation);
    public static final Pose2d kBlueNine = new Pose2d(kBlueXm, kBlueNineYm, kBlueScoreRotation);

    public static final double kTagXtoScoreX = 0.85;
    public static final double kTagYtoScoreY = 0.65;
    public static final Transform2d kTransformTagCube =
            new Transform2d(new Translation2d(kTagXtoScoreX, 0), kZero);
    public static final Transform2d kTransformTagConeLeft =
            new Transform2d(new Translation2d(kTagXtoScoreX, kTagYtoScoreY), kZero);
    public static final Transform2d kTransformTagConeRight =
            new Transform2d(new Translation2d(kTagXtoScoreX, -kTagYtoScoreY), kZero);

    public static final Pose2d kGroundIntake = new Pose2d();

    public static final double kScoreTargetHeightMeters = 0.4318;
    public static final double kIntakeTargetHeightMeters = 0;

    public static Translation2d flipTranslation(Translation2d translation) {
        return new Translation2d(
                FieldConstants.kFieldLength - translation.getX(), translation.getY());
    }

    public static Pose2d flipBluePose(Pose2d pose) {
        return new Pose2d(
                flipTranslation(pose.getTranslation()),
                new Rotation2d(pose.getRotation().getCos() * -1, pose.getRotation().getSin()));
    }

    public static Transform2d flipBlueTransform(Transform2d transform) {
        return new Transform2d(kOrigin, flipBluePose(kOrigin.transformBy(transform)));
    }

    private FieldConstants() {}
}
