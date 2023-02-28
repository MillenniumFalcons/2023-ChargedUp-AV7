// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team3647.frc2023.constants.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public final class AllianceFlipUtil {
    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(
                    FieldConstants.kFieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    FieldConstants.kFieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    private AllianceFlipUtil() {}
}
