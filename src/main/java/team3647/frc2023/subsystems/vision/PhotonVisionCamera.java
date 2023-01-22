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
import org.photonvision.targeting.PhotonTrackedTarget;
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
        List<PhotonTrackedTarget> targets = null;

        boolean hasTarget = false;
        double lastTimestamp = 0.0;
        int bestTagID = -1;
        double bestPoseAmbiguity = 0;
        double bestAvgYaw = 0.0;
        double bestAvgPitch = 0.0;
        double bestAvgArea = 0.0;
        double bestAvgSkew = 0.0;
        double bestCameraToTag = 0.0;
        double bestCameraToTagAngle = 0.0;
        double bestCameraToTagY = 0.0;
        double bestCameraToTagX = 0.0;
        Transform3d bestAvgPose = null;
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
        var results = camera.getLatestResult();
        var bestResult = results.getBestTarget();
        periodicIO.hasTarget = camera.getLatestResult().hasTargets();
        if (camera.getLatestResult().hasTargets()) {
            periodicIO.targets = results.targets;
            periodicIO.bestTagID = bestResult.getFiducialId();
            periodicIO.bestPoseAmbiguity = bestResult.getPoseAmbiguity();
            periodicIO.bestAvgYaw = bestResult.getYaw();
            periodicIO.bestAvgPose = bestResult.getBestCameraToTarget();
            periodicIO.bestCameraToTagAngle = Units.degreesToRadians(bestResult.getYaw());
            periodicIO.bestCameraToTagX = periodicIO.bestAvgPose.getX() * Math.cos(Units.degreesToRadians(bestResult.getYaw()));
            periodicIO.bestCameraToTagY = periodicIO.bestAvgPose.getX() * Math.sin(Units.degreesToRadians((bestResult.getYaw())));
        }
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        return periodicIO.targets;
    }

    public double getCameraToTag(){
        return periodicIO.bestCameraToTag;
    }

    public double getCameraToTagAngle(){
        return periodicIO.bestCameraToTagAngle;
    }

    public double getCameraToTagX() {
        return periodicIO.bestCameraToTagX;
    }

    public double getCameraToTagY() {
        return periodicIO.bestCameraToTagY;
    }

    public Transform3d getCameraToTagTransform() {
        return periodicIO.bestAvgPose;
    }

    public boolean getHasTarget() {
        return periodicIO.hasTarget;
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