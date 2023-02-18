package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class StampedPose {
    public final Pose2d pose;
    public final double timestamp;

    public StampedPose(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
