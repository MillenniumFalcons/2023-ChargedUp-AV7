package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import team3647.lib.vision.IVisionCamera.CamConstants;

public class LimelightConstant {
    public final static double kCameraHeightMeters = 0.49;
    public final static Rotation2d kHorizontalToLens = new Rotation2d();
    public final static double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public final static double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public final static CamConstants kCamConstatnts =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kVPH, kVPW);
}
