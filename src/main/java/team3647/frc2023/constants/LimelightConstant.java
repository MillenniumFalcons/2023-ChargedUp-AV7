package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.IVisionCamera.VisionPipeline;

public class LimelightConstant {

    public static final String kLimelightCenterIP = "10.36.47.16";
    public static final String kLimelightCenterHost = "limelight-center";

    public static final String kLimelightRightIP = "10.36.47.15";
    public static final String kLimelightRightHost = "limelight-right";

    public static final String kLimelightLeftIP = "10.36.47.14";
    public static final String kLimelightLeftHost = "limelight-left";

    public static final double kCameraHeightMeters = 0.49;
    public static final Rotation2d kHorizontalToLens = new Rotation2d();
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public static final CamConstants kCamConstatnts =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kVPH, kVPW);

    public static final Transform2d kRobotToCamFixed = new Transform2d();

    public static final VisionPipeline APRIL_PIPELINE = new VisionPipeline(0, 2592, 1944);
    public static final VisionPipeline TAPE_PIPELINE = new VisionPipeline(1, 1280, 960);

    private LimelightConstant() {}
}
