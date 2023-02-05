package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagCamera extends IVisionCamera {
    public static final StampedPose KNoAnswer = new StampedPose(new Pose2d(), 0);

    public StampedPose getRobotPose();
}
