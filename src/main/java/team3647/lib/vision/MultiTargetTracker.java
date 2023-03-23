package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

public class MultiTargetTracker {
    private final Map<AprilTagId, TrackedTarget> trackedTargets = new EnumMap<>(AprilTagId.class);

    public void update(double timestamp, AprilTagId id, Pose2d fieldToGoal) {

        if (fieldToGoal == null) {
            return;
        }

        if (!trackedTargets.containsKey(id)) {
            System.out.printf("Found new target %d%n", id.num);
            trackedTargets.put(id, new TrackedTarget(timestamp, id, fieldToGoal));
        } else {
            SmartDashboard.putNumber("seeing target", id.num);
            trackedTargets.get(id).update(timestamp, fieldToGoal);
        }

        trackedTargets.values().forEach(TrackedTarget::removeOldObservations);

        removeDeadTargets();
    }

    public void removeDeadTargets() {
        for (var entry : trackedTargets.entrySet()) {
            entry.getValue().removeOldObservations();
            if (!entry.getValue().isAlive()) {
                trackedTargets.remove(entry.getKey());
            }
        }
    }

    public boolean hasTracks() {
        return !trackedTargets.isEmpty();
    }

    public List<TrackedTarget> getTrackedTargets() {
        return trackedTargets.values().stream().toList();
    }

    public TrackedTarget getById(AprilTagId id) {
        if (trackedTargets.containsKey(id)) {
            return trackedTargets.get(id);
        }
        return null;
    }
}
