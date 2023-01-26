package team3647.frc2023.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.Limelight.Data;

public class LimelightController implements PeriodicSubsystem {

    private final Limelight limelight = new Limelight(PhotonVisionConstants.kLimelightIP, 0, LimelightConstant.kCamConstatnts);
    private PeriodicIO periodicIO = new PeriodicIO();

    private static final class PeriodicIO {
        boolean hasTarget = false;
        double latency = 0.0;
        int tagID = -1;
        double poseAmbiguity = 0;
        double avgYaw = 0.0;
        double avgPitch = 0.0;
        double avgArea = 0.0;
        double avgSkew = 0.0;
        double cameraToTag = 0.0;
        double cameraToTagAngle = 0.0;
        double cameraToTagY = 0.0;
        double cameraToTagX = 0.0;
        double[] rawRobotPoseArray = {};
        
    }

    public LimelightController() {
    }

    @Override
    public void readPeriodicInputs() {
        //FIX cause null pointer error
        if(limelight.getDouble(Data.VALID_TARGET) == 1) {
            periodicIO.hasTarget = true;
            periodicIO.avgYaw = limelight.getDoubleArray(Data.CAM_POSE)[4];
            periodicIO.avgPitch = limelight.getDoubleArray(Data.CAM_POSE)[3];
            periodicIO.avgSkew = limelight.getDoubleArray(Data.CAM_POSE)[5];
            periodicIO.avgArea = limelight.getDouble(Data.AREA);
            periodicIO.latency = limelight.getDouble(Data.LATNECY_MS);
            periodicIO.tagID = (int) limelight.getDouble(Data.TAG_ID);
            periodicIO.cameraToTagX = -limelight.getDoubleArray(Data.CAM_POSE)[2];
            periodicIO.cameraToTagY = -limelight.getDoubleArray(Data.CAM_POSE)[0];    
            periodicIO.rawRobotPoseArray = limelight.getDoubleArray(Data.ROBOT_POSE);
        } else {
            periodicIO.hasTarget = false;
            periodicIO.tagID = -1;
        }
    }

    public double[] getRobotPoseArray() {
        return periodicIO.rawRobotPoseArray;
    }

    public boolean hasTarget() {
        return periodicIO.hasTarget;
    }

    public double getYaw() {
        return periodicIO.avgYaw;
    }

    public double getPitch() {
        return periodicIO.avgPitch;
    }

    public double getLatency() {
        return periodicIO.latency;
    }

    public double getTagID() {
        return periodicIO.tagID;
    }

    public double getCameraToTagX() {
        return periodicIO.cameraToTagX;
    }

    public double getCameraToTagY() {
        return periodicIO.cameraToTagY;
    }

    public Translation3d getCameraToTag() {
        return new Translation3d(periodicIO.cameraToTagX, periodicIO.cameraToTagY, 0);
    }

    public double getDistance() {
        return Math.sqrt(periodicIO.cameraToTagX * periodicIO.cameraToTagX + periodicIO.cameraToTagY * periodicIO.cameraToTagY);
    }

    @Override
    public String getName() {
        return "Limelight";
    }
    
}
