package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import team3647.lib.vision.IVisionCamera.CamConstants;

public class LimelightConstant {

    public static final String kLimelightCenterIP = "10.36.47.16";
    public static final String kLimelightCenterHost = "limelight-center";

    public static final String kLimelightRightIP = "10.36.47.15";
    public static final String kLimelightRightHost = "limelight-right";

    public static final String kLimelightLeftIP = "10.36.47.14";
    public static final String kLimelightLeftHost = "limelight-left";

    public static final double kCameraHeightMeters =
            Units.inchesToMeters(7.08); // Units.inchesToMeters(8.75);
    public static final Rotation2d kHorizontalToLens = Rotation2d.fromDegrees(27.7);
    public static final Rotation2d kCameraRoll = Rotation2d.fromDegrees(0.0);
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public static final CamConstants kCamConstants =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kCameraRoll, kVPH, kVPW);

    public static final Transform2d kRobotToCamFixed =
            new Transform2d(new Translation2d(0.311, 0), new Rotation2d());

    private LimelightConstant() {}
}
