package team3647.frc2023.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Map;

public class Scoring {
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT
    };

    public static class Section {
        public final Translation2d centerPt;
        private final Map<Position, Pose2d> actualPose;

        public Section(Translation2d centerPt, Map<Position, Pose2d> actualPose) {
            this.centerPt = centerPt;
            this.actualPose = actualPose;
        }

        public Pose2d getPose(Position position) {
            return actualPose.get(position);
        }
    }

    public static Section getClosest(List<Section> sections, Translation2d point) {
        var smallest = sections.get(0);
        var smallestNorm = Double.MAX_VALUE;
        for (var s : sections) {
            var dist = s.centerPt.getDistance(point);
            if (dist < smallestNorm) {
                smallest = s;
                smallestNorm = dist;
            }
        }

        return smallest;
    }

    private Scoring() {}
}
