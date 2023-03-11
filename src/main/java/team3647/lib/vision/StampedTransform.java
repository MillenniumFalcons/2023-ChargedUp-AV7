package team3647.lib.vision;

import edu.wpi.first.math.geometry.Transform2d;

public class StampedTransform {
    public final Transform2d transform;
    public final double timestamp;

    public StampedTransform(Transform2d transform, double timestamp) {
        this.transform = transform;
        this.timestamp = timestamp;
    }
}
