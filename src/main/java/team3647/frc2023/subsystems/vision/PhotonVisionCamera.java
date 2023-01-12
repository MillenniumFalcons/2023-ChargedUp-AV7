package team3647.frc2023.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.TargetCorner;

import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.lib.PeriodicSubsystem;

public class PhotonVisionCamera implements PeriodicSubsystem {
    private final PhotonCamera camera;
    private RobotPoseEstimator robotPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final PeriodicIO periodicIO = new PeriodicIO();
    private final AprilTag tag01 =
                new AprilTag(
                        1,
                        new Pose3d(new Pose2d(0, Units.inchesToMeters(0), Rotation2d.fromDegrees(0.0))));
    private final ArrayList<AprilTag> atList = new ArrayList<AprilTag>();

    private static final class PeriodicIO {
        double lastTimestamp = 0.0;
        int tagID = -1;
        double poseAmbiguity = 0;
        double avgYaw = 0.0;
        double avgPitch = 0.0;
        double avgArea = 0.0;
        double avgSkew = 0.0;
        double cameraToTagY = 0.0;
        double cameraToTagX = 0.0;
        Transform3d avgPose = null;
        List<TargetCorner> corners = null;
    }

    public PhotonVisionCamera(PhotonCamera camera) {
        this.camera = camera;
        this.atList.add(tag01);
        aprilTagFieldLayout = new AprilTagFieldLayout(atList, 6.09, 3.00);
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(this.camera, PhotonVisionConstants.robotToCam));
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    @Override
    public void readPeriodicInputs() {
        PeriodicSubsystem.super.readPeriodicInputs();
        var result = camera.getLatestResult().getBestTarget();
        if (camera.getLatestResult().hasTargets()) {
            periodicIO.tagID = result.getFiducialId();
            periodicIO.poseAmbiguity = result.getPoseAmbiguity();
            periodicIO.avgYaw = result.getYaw();
            periodicIO.avgPose = result.getBestCameraToTarget();
            periodicIO.cameraToTagX = periodicIO.avgPose.getX() * Math.cos(Units.degreesToRadians(result.getYaw()));
            periodicIO.cameraToTagY = periodicIO.avgPose.getX() * Math.sin(Units.degreesToRadians((result.getYaw())));
            SmartDashboard.putNumber("Distance", periodicIO.avgPose.getX());
            SmartDashboard.putNumber("Angle to Target", periodicIO.avgYaw);
            SmartDashboard.putNumber("camera to tag y", periodicIO.cameraToTagY);
            SmartDashboard.putNumber("camera to tag x", periodicIO.cameraToTagX);
        }
    }

    public double getCameraToTagX() {
        return periodicIO.cameraToTagX;
    }

    public double getCameraToTagY() {
        return periodicIO.cameraToTagY;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            try {
                return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
            } catch (NullPointerException e){
                return new Pair<Pose2d, Double>(null, 0.0);
            }
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    @Override
    public String getName() {
        return null;
    }
}