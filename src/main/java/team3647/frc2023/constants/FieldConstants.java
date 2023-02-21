// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Map;
import team3647.frc2023.util.Scoring;
import team3647.lib.GroupPrinter;

/** Add your docs here. */
public class FieldConstants {
    public static final GroupPrinter printer = GroupPrinter.getInstance();
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);

    // offset up down, negative down, positive up
    public static final double yOffsetInches = 0.0; // -10.0;
    // offset towards or away from the scoring stations, positive farther towards center of field,
    // negative closer towards scoring stations
    public static final double xOffsetInches = 4.0;
    public static final double blueXLineUpDistance = Units.inchesToMeters(70.5);

    public static final Translation2d oneBlueT =
            new Translation2d(blueXLineUpDistance, Units.inchesToMeters(42));
    public static final Translation2d twoBlueT =
            new Translation2d(blueXLineUpDistance, Units.inchesToMeters(108));
    public static final Translation2d threeBlueT =
            new Translation2d(blueXLineUpDistance, Units.inchesToMeters(174));

    public static final Translation2d oneRedT = flip(oneBlueT);
    public static final Translation2d twoRedT = flip(twoBlueT);
    public static final Translation2d threeRedT = flip(threeBlueT);

    public static final Scoring.Section oneBlue =
            new Scoring.Section(
                    oneBlueT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(196 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(152 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(174 + yOffsetInches)),
                                    new Rotation2d())));
    public static final Scoring.Section twoBlue =
            new Scoring.Section(
                    twoBlueT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(130 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(86 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(108 + yOffsetInches)),
                                    new Rotation2d())));
    public static final Scoring.Section threeBlue =
            new Scoring.Section(
                    threeBlueT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(64 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(20 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(70.5 + xOffsetInches),
                                            Units.inchesToMeters(42 + yOffsetInches)),
                                    new Rotation2d())));
    public static final List<Scoring.Section> allBlue = List.of(oneBlue, twoBlue, threeBlue);
    public static final Scoring.Section oneRed =
            new Scoring.Section(
                    oneRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(20 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(64 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(42 + yOffsetInches)),
                                    new Rotation2d())));
    public static final Scoring.Section twoRed =
            new Scoring.Section(
                    twoRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(86 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(130 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(108 + yOffsetInches)),
                                    new Rotation2d())));
    public static final Scoring.Section threeRed =
            new Scoring.Section(
                    threeRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(152 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.RIGHT,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(196 + yOffsetInches)),
                                    new Rotation2d()),
                            Scoring.Position.MIDDLE,
                            new Pose2d(
                                    new Translation2d(
                                            Units.inchesToMeters(580.5 - xOffsetInches),
                                            Units.inchesToMeters(174 + yOffsetInches)),
                                    new Rotation2d())));
    public static final List<Scoring.Section> allRed = List.of(oneRed, twoRed, threeRed);

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
                FieldConstants.fieldLength - translation.getX(), translation.getY());
    }

    static {
        printer.addPose("1blueT", () -> new Pose2d(oneBlueT, new Rotation2d()));
        printer.addPose("2blueT", () -> new Pose2d(twoBlueT, new Rotation2d()));
        printer.addPose("3blueT", () -> new Pose2d(threeBlueT, new Rotation2d()));

        printer.addPose("1redT", () -> new Pose2d(oneRedT, new Rotation2d()));
        printer.addPose("2redT", () -> new Pose2d(twoRedT, new Rotation2d()));
        printer.addPose("3redT", () -> new Pose2d(threeRedT, new Rotation2d()));
    }

    private FieldConstants() {}
}
