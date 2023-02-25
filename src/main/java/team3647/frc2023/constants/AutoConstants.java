package team3647.frc2023.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxRotSpeedRadPerSec =
            SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec;

    public static final double kMaxAccelerationMetersPerSecSq = 3;
    public static final double kMaxRotAccelerationRadsPerSecSq = Math.PI;

    public static final double kXControllerP = 5; // 1.4, 2.2;
    public static final double kXControllerI = 0; // 6;
    public static final double kXControllerD = 0; // 0;

    public static final double kYControllerP = 5; // 1.4, 2.2;
    public static final double kYControllerI = 0; // 6;
    public static final double kYControllerD = 0; // 0;

    public static final double xRotControllerP = 11; // 12;
    public static final double xRotControllerI = 4; // 8;
    public static final double xRotControllerD = 0;

    public static final TrapezoidProfile.Constraints kRotControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxRotSpeedRadPerSec, kMaxRotAccelerationRadsPerSecSq);

    public static final PIDController kXController =
            new PIDController(kXControllerP, kXControllerI, kXControllerD);
    public static final PIDController kYController =
            new PIDController(kYControllerP, kYControllerI, kYControllerD);
    public static final PIDController kRotController =
            new PIDController(xRotControllerP, xRotControllerI, xRotControllerD);

    static {
        kRotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private AutoConstants() {}
}
