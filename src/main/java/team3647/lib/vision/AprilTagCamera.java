package team3647.lib.vision;

public interface AprilTagCamera extends IVisionCamera {
    public enum AprilTagId {
        ID_1(1),
        ID_2(2),
        ID_3(3),
        ID_4(4),
        ID_5(5),
        ID_6(6),
        ID_7(7),
        ID_8(8),
        ID_9(9),
        ID_DNE(-1);

        public final int num;

        AprilTagId(int num) {
            this.num = num;
        }
    }

    public static AprilTagId getId(int id) {
        var adjustedId = id - 1;
        var possibleValues = AprilTagId.values();
        if (adjustedId < 0 || adjustedId > possibleValues.length) {
            return AprilTagId.ID_DNE;
        }

        return possibleValues[adjustedId];
    }

    public static class VisionUpdate {
        public final double captureTimestamp;
        public final VisionPoint point;
        public final AprilTagId id;

        public VisionUpdate(double captureTimestamp, VisionPoint point, AprilTagId id) {
            this.captureTimestamp = captureTimestamp;
            this.point = point;
            this.id = id;
        }

        public static final VisionUpdate kNoUpdate =
                new VisionUpdate(0, new VisionPoint(0, 0), AprilTagId.ID_DNE);
    }

    public VisionUpdate getVisionUpdate();
}
