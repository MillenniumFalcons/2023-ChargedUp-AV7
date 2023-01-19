package team3647.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.Objects;
import team3647.lib.utils.NamedInt;

public interface IVisionCamera {

    public static class VisionInputs {
        public double captureTimestamp = 0.0;
        public List<VisionPoint> corners;
        public boolean validEntry = false;
        public double skew = 0.0;
        public double angleToVisionCenter = 0;
        public double pitchToVisionCenter = 0;
    }

    public static class LEDMode extends NamedInt {
        public LEDMode(int ntID) {
            super(ntID);
        }
    }

    public static class CamMode extends NamedInt {
        public CamMode(int ntID) {
            super(ntID);
        }
    }

    public static final class VisionPipeline extends NamedInt {
        public final int width, height;

        public VisionPipeline(int ntID, int widthPixels, int heightPixels) {
            super(ntID);
            this.width = widthPixels;
            this.height = heightPixels;
        }
    }

    public static class VisionPoint {
        public final double x, y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class CamConstants {
        public final double kCameraHeightMeters;
        public final Rotation2d kHorizontalToLens;
        public final double kVPH;
        public final double kVPW;

        public CamConstants(
                double kCameraHeightMeters,
                Rotation2d kHorizontalToLens,
                double kVPH,
                double kVPW) {
            this.kCameraHeightMeters = kCameraHeightMeters;
            this.kHorizontalToLens = Objects.requireNonNull(kHorizontalToLens);
            this.kVPH = kVPH;
            this.kVPW = kVPW;
        }
    }

    public void writeToInputs(VisionInputs inputs);

    public void setLED(LEDMode ledMode);

    public void setCamMode(CamMode camMode);

    public void setPipeline(VisionPipeline pipeline);

    public Rotation2d getHorizontalPlaneToLens();

    public double getLensHeightMeters();

    public double getVPW();

    public double getVPH();

    public String toString();

    public CamConstants getConstants();

    public VisionPipeline getPipeline();
}
