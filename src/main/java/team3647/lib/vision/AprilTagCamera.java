package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface AprilTagCamera extends IVisionCamera {
    public enum AprilTagId {
        ID_1,
        ID_2,
        ID_3,
        ID_4,
        ID_5,
        ID_6,
        ID_7,
        ID_8,
        ID_9
    }

    public static final StampedPose KNoAnswer = new StampedPose(new Pose2d(), 0);

    public StampedPose getRobotPose();
}
