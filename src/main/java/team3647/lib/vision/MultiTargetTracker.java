package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import java.util.Map;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

public class MultiTargetTracker {
    private final Map<AprilTagId, TrackedTarget> trackedTargets = new HashMap<>();

    public void update(double timestamp, AprilTagId id, Pose2d fieldToGoal) {

        if (!trackedTargets.containsKey(id)) {
            System.out.println("Found new target");
            trackedTargets.put(id, new TrackedTarget(timestamp, id, fieldToGoal));
        } else {
            trackedTargets.get(id).attemptUpdate(timestamp, fieldToGoal);
        }

        trackedTargets.values().forEach(TrackedTarget::removeOldObservations);
    }

    public boolean hasTracks() {
        return !trackedTargets.isEmpty();
    }

    public Map<AprilTagId, TrackedTarget> getTrackedTargets() {
        return trackedTargets;
    }
}
