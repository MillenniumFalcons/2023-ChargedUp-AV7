package team3647.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;

public class PhotonVisionConstants {
    public static final String kLimelightIP = "10.36.47.15";
    public static final double kNetworkLatency = 0.06; // seconds
    // public static Transform3d robotToCam = new Transform3d(new Translation3d(0, -0, 0), new
    // Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a
    // meter up from center.
    public static final Transform3d robotToCam =
            new Transform3d(
                    new Translation3d(0.12, -0.27, 0.0),
                    new Rotation3d(
                            0, 0,
                            0)); // Cam mounted facing forward, half a meter forward of center, half
    // a meter up from center.
    public static final PhotonCamera camera = new PhotonCamera("gloworm");

    private PhotonVisionConstants() {}
}
