// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
    public static final PathPoint middle_cones =
            new PathPoint(new Translation2d(12.75 + 1.9, 4.3 - 1.7), new Rotation2d(180.0));

    private FieldConstants() {}
}
