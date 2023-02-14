// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Map;
import team3647.frc2023.util.Scoring;

/** Add your docs here. */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);

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
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final Scoring.Section twoBlue =
            new Scoring.Section(
                    twoBlueT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final Scoring.Section threeBlue =
            new Scoring.Section(
                    threeBlueT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final List<Scoring.Section> allBlue = List.of(oneBlue, twoBlue, threeBlue);
    public static final Scoring.Section oneRed =
            new Scoring.Section(
                    oneRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final Scoring.Section twoRed =
            new Scoring.Section(
                    twoRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final Scoring.Section threeRed =
            new Scoring.Section(
                    threeRedT,
                    Map.of(
                            Scoring.Position.LEFT,
                            new Pose2d(),
                            Scoring.Position.RIGHT,
                            new Pose2d(),
                            Scoring.Position.MIDDLE,
                            new Pose2d()));
    public static final List<Scoring.Section> allRed = List.of(oneRed, twoRed, threeRed);

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
                FieldConstants.fieldLength - translation.getX(), translation.getY());
    }

    private FieldConstants() {}
}
