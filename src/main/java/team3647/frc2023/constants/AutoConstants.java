package team3647.frc2023.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
    public static double kMaxSpeedMetersPerSecond = 2;
    public static double kMaxRotSpeedRadPerSec = SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec;

    public static double kMaxAccelerationMetersPerSecSq = 2;
    public static double kMaxRotAccelerationRadsPerSecSq = Math.PI;

    public static final double kXControllerP = 1.4; // 2.2;
    public static final double kXControllerI = 0; // 6;
    public static final double kXControllerD = 0; // 0;

    public static final double kYControllerP = 1.4; // 2.2;
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
