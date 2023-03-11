package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;

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
        ID_9,
        ID_DNE
    }

    public static final StampedPose KNoAnswer = new StampedPose(new Pose2d(), 0);
    public static final Map<AprilTagId, StampedTransform> kNoTags = Map.of();

    public StampedPose getRobotPose();

    public Map<AprilTagId, StampedTransform> getCamToTags();

    public static AprilTagId getId(int id) {
        var adjustedId = id - 1;
        var possibleValues = AprilTagId.values();
        if (adjustedId < 0 || adjustedId > possibleValues.length) {
            return AprilTagId.ID_DNE;
        }

        return possibleValues[adjustedId];
    }
}
