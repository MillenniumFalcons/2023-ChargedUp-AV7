package team3647.frc2023.constants;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {
    public static String kLimelightIP = "10.36.47.15";
    public static double kNetworkLatency = 0.06; // seconds
    // public static Transform3d robotToCam = new Transform3d(new Translation3d(0, -0, 0), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static Transform3d robotToCam = new Transform3d(new Translation3d(0.12, -0.27, 0.0), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static double offsetAprilTagToCenterOfRobotFlush = Units.inchesToMeters(32);
    public static final PhotonCamera camera = new PhotonCamera("gloworm");
}