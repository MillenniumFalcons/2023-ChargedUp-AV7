package team3647.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import team3647.lib.utils.NamedInt;

public class Limelight implements AprilTagCamera {

    // NetworkTable is the class used to grab values from the Limelight Network
    // Table
    private final NetworkTable table;
    private final double[] emptyDoubleArray = {};
    private final double extraLatencySec;
    private final String ip;
    private final CamConstants kCamConstants;

    private VisionPipeline currentPipeline = new VisionPipeline(0, 1280, 960);

    public enum Data {
        VALID_TARGET("tv"),
        X("tx"),
        Y("ty"),
        AREA("ta"),
        LATENCY_PIPE_MS("tl"),
        LATENCY_CAP_MS("cl"),
        TAG_ID("tid"),
        CORNERS("tcornxy"),
        ROBOT_POSE("botpose_wpiblue"),
        TAG_POSE("targetpose_cameraspace");

        public final String str;

        Data(String str) {
            this.str = str;
        }
    }

    // used to initalize the main, important things
    public Limelight(String ip, double extraLatencySec, CamConstants camConstants) {
        this(ip, "limelight", extraLatencySec, camConstants);
    }

    public Limelight(String ip, String name, double extraLatencySec, CamConstants camConstants) {
        // initializing the network table to grab values from limelight
        table = NetworkTableInstance.getDefault().getTable(name);
        this.ip = ip;
        this.extraLatencySec = extraLatencySec;
        this.kCamConstants = camConstants;
        table.getEntry(Data.LATENCY_PIPE_MS.str);
        table.getEntry(Data.LATENCY_CAP_MS.str);
    }

    public synchronized void writeToInputs(VisionInputs inputs) {}

    @Override
    public void setLED(LEDMode ledMode) {
        set("ledMode", ledMode);
    }

    @Override
    public void setCamMode(CamMode camMode) {
        set("camMode", camMode);
    }

    @Override
    public void setPipeline(VisionPipeline pipeline) {
        this.currentPipeline = pipeline;
        set("pipeline", pipeline);
    }

    private void set(String input, NamedInt input2) {
        table.getEntry(input).setNumber(input2.asInt);
    }

    public double[] getDoubleArray(Data key) {
        return table.getEntry(key.str).getDoubleArray(emptyDoubleArray);
    }

    public double getDouble(Data key) {
        return table.getEntry(key.str).getDouble(0.0);
    }

    public String toString() {
        return ip;
    }

    @Override
    public Rotation2d getHorizontalPlaneToLens() {
        return kCamConstants.kHorizontalToLens;
    }

    @Override
    public double getLensHeightMeters() {
        return kCamConstants.kCameraHeightMeters;
    }

    @Override
    public double getVPW() {
        return kCamConstants.kVPW;
    }

    @Override
    public double getVPH() {
        return kCamConstants.kVPH;
    }

    @Override
    public CamConstants getConstants() {
        return kCamConstants;
    }

    @Override
    public VisionPipeline getPipeline() {
        return currentPipeline;
    }

    @Override
    public VisionUpdate getVisionUpdate() {
        var isTarget = getDouble(Data.VALID_TARGET);
        if (isTarget == 0) {
            return VisionUpdate.kNoUpdate;
        }
        var tagIdInt = (int) getDouble(Data.TAG_ID);
        if (tagIdInt == -1) {
            return VisionUpdate.kNoUpdate;
        }
        AprilTagId id = AprilTagCamera.getId(tagIdInt);
        if (id == AprilTagId.ID_DNE) {
            return VisionUpdate.kNoUpdate;
        }

        var tx = getDouble(Data.X);
        var ty = getDouble(Data.Y);
        var totalTime =
                getDouble(Data.LATENCY_CAP_MS) / 1000.0 + getDouble(Data.LATENCY_PIPE_MS) / 1000;
        return new VisionUpdate(
                Timer.getFPGATimestamp() - totalTime - extraLatencySec,
                new VisionPoint(tx, ty),
                id);
    }
}
