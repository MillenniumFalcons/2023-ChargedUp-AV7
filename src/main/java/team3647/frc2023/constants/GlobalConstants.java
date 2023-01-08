package team3647.frc2023.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
    public static final double kFalconTicksPerRotation = 2048;
    public static final PneumaticsModuleType kPCMType = PneumaticsModuleType.CTREPCM;
    public static final double kDt = 0.02;
    public static final int kTimeoutMS = 255;
    public static double kCenterOffsetMeters = Units.inchesToMeters(34);
    public static double kDistanceTarmacToGoalCenter = Units.inchesToMeters(84);
    public static double kDistanceFarToGoalCenter = Units.feetToMeters(14) + kCenterOffsetMeters;

    public static final class SwerveDriveIds {
        public static final int kFrontLeftDriveId = 1;
        public static final int kFrontLeftTurnId = 2;
        public static final int kFrontLeftAbsEncoderPort = 9;

        public static final int kFrontRightDriveId = 3;
        public static final int kFrontRightTurnId = 4;
        public static final int kFrontRightAbsEncoderPort = 10;

        public static final int kBackLeftDriveId = 5;
        public static final int kBackLeftTurnId = 6;
        public static final int kBackLeftAbsEncoderPort = 11;

        public static final int kBackRightDriveId = 7;
        public static final int kBackRightTurnId = 8;
        public static final int kBackRightAbsEncoderPort = 12;

        public static final int gyroPin = 20;
    }

    public static final class IntakeIds {
        public static final int kIntakeMotorId = 13;
        public static final int kDeployMotorId = 14;
    }

    public static final class ColumnIds {
        public static final int kMasterMotorId = 15;
        public static final int kTopSensorPin = 4;
    }

    public static final class TurretIds {
        public static final int kMotorId = 16;
    }

    public static final class HoodIds {
        public static final int kMotorId = 17;
    }

    public static final class FlywheelIds {
        public static final int kMasterId = 18;
        public static final int kFollowerId = 19;
    }

    private GlobalConstants() {}
}
