package team3647.frc2023.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.frc2023.subsystems.vision.PhotonVisionCamera;

public class Superstructure {
    private final SwerveDrive drive;

    public Superstructure(SwerveDrive drive) {
        this.drive = drive;
    }

    public void periodic(double timestamp) {
            // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
            
    }
}


