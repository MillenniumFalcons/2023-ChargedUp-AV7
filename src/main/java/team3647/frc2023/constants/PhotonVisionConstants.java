package team3647.frc2023.constants;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionConstants {
    public static String kLimelightIP = "10.36.47.15";
    public static double kNetworkLatency = 0.06; // seconds
    public static Transform3d robotToCam = new Transform3d(new Translation3d(0.1, 0.27, 0.49), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final PhotonCamera camera = new PhotonCamera("gloworm");
}