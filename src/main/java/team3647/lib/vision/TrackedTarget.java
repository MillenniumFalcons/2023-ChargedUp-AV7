package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Comparator;
import java.util.TreeMap;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

/** 254 GoalTrack class */
public class TrackedTarget {
    private static final double kCamFrameRate = 40;
    private static final double kMaxTrackedTargetAge = 1; // seconds
    private static final double kMaxTrackedTargetSmoothingTime = 1;
    private static final double kMaxTrackerDistance = 0.20;

    public final AprilTagId id;
    private final TreeMap<Double, Pose2d> observedPositions = new TreeMap<>();
    private Pose2d smoothedPosition;

    public TrackedTarget(double timestamp, AprilTagId id, Pose2d firstObserved) {
        this.observedPositions.put(timestamp, firstObserved);
        this.id = id;
        this.smoothedPosition = firstObserved;
    }

    public void update(double timestamp, Pose2d newObserved) {
        removeOldObservations();
        observedPositions.put(timestamp, newObserved);
        smoothedPosition = smoothPosition();
    }

    public boolean isAlive() {
        return !observedPositions.isEmpty();
    }

    public Pose2d getSmoothedPosition() {
        return smoothedPosition;
    }

    public double getStability() {
        return Math.min(1.0, observedPositions.size() / (kCamFrameRate * kMaxTrackedTargetAge));
    }

    public double getLatestTimestamp() {
        return observedPositions.keySet().stream().max(Double::compare).orElse(0.0);
    }

    public void removeOldObservations() {
        double threshold = Timer.getFPGATimestamp() - kMaxTrackedTargetAge;
        observedPositions.entrySet().removeIf(entry -> entry.getKey() < threshold);
    }

    private Pose2d smoothPosition() {
        if (!isAlive()) {
            return null;
        }
        double x = 0;
        double y = 0;
        double cos = 0;
        double sin = 0;
        double timestamp = Timer.getFPGATimestamp();
        int numSamples = 0;
        for (var entry : observedPositions.entrySet()) {
            if (timestamp - entry.getKey() > kMaxTrackedTargetSmoothingTime) {
                continue;
            }
            ++numSamples;
            x += entry.getValue().getX();
            y += entry.getValue().getY();
            cos += entry.getValue().getRotation().getCos();
            sin += entry.getValue().getRotation().getSin();
        }
        if (numSamples == 0) {
            return observedPositions.lastEntry().getValue();
        }
        x /= numSamples;
        y /= numSamples;
        cos /= numSamples;
        sin /= numSamples;
        return new Pose2d(x, y, new Rotation2d(cos, sin));
    }

    public static class TrackedTargetComparator implements Comparator<TrackedTarget> {
        private final double stabilityWeight;
        private final double ageWeight;
        private final double currentTimestamp;
        private final double switchingWeight;
        private AprilTagId lastTargetId;

        public TrackedTargetComparator(
                double stabilityWeight,
                double ageWeight,
                double currentTimestamp,
                double switchingWeight,
                AprilTagId lastTargetId) {
            this.stabilityWeight = stabilityWeight;
            this.ageWeight = ageWeight;
            this.currentTimestamp = currentTimestamp;
            this.switchingWeight = switchingWeight;
            this.setLastTargetId(lastTargetId);
        }

        public void setLastTargetId(AprilTagId lastTargetId) {
            this.lastTargetId = lastTargetId;
        }

        private double score(TrackedTarget target) {
            double stabilityScore = stabilityWeight * target.getStability();
            double ageScore =
                    ageWeight
                            * Math.max(
                                    0,
                                    (kMaxTrackedTargetAge
                                            - (currentTimestamp - target.getLatestTimestamp())
                                                    / kMaxTrackedTargetAge));
            double switchingScore = target.id == lastTargetId ? switchingWeight : 0;
            return stabilityScore + ageScore + switchingScore;
        }

        @Override
        public int compare(TrackedTarget arg0, TrackedTarget arg1) {
            double diff = score(arg0) - score(arg1);
            if (diff < 0) {
                return 1;
            } else if (diff > 0) {
                return -1;
            }
            return 0;
        }
    }
}
