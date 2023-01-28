package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import team3647.lib.vision.IVisionCamera.CamConstants;

public class LimelightConstant {
    public static final double kCameraHeightMeters = 0.49;
    public static final Rotation2d kHorizontalToLens = new Rotation2d();
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public static final CamConstants kCamConstatnts =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kVPH, kVPW);
}
