package team3647.frc2023.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.lib.PeriodicSubsystem;

public class PhotonVisionCamera implements PeriodicSubsystem {
    private final PhotonCamera camera;
    private RobotPoseEstimator robotPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final PeriodicIO periodicIO = new PeriodicIO();

    private static final class PeriodicIO {
        double lastTimestamp = 0.0;
        int tagID = -1;
        double poseAmbiguity = 0;
        double avgYaw = 0.0;
        double avgPitch = 0.0;
        double avgArea = 0.0;
        double avgSkew = 0.0;
        Transform3d avgPose = null;
        List<TargetCorner> corners = null;
    }

    public PhotonVisionCamera(PhotonCamera camera) {
        this.camera = camera;
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(this.camera, PhotonVisionConstants.robotToCam));

        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    @Override
    public void readPeriodicInputs() {
        PeriodicSubsystem.super.readPeriodicInputs();
        var result = camera.getLatestResult().getBestTarget();
        if (camera.getLatestResult().hasTargets()) {
            periodicIO.tagID = result.getFiducialId();
            periodicIO.poseAmbiguity = result.getPoseAmbiguity();
            periodicIO.avgPitch = result.getPitch();
            periodicIO.avgYaw = result.getYaw();
            periodicIO.avgSkew = result.getSkew();
            periodicIO.avgArea = result.getArea();
            periodicIO.avgPose = result.getBestCameraToTarget();
            SmartDashboard.putNumber("X ROBOT TO CAM", periodicIO.avgPose.getX());
        }
    }

    public double getAvgYaw() {
        return periodicIO.avgYaw;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}