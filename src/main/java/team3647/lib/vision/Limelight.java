package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import team3647.lib.utils.NamedInt;
import team3647.lib.vision.IVisionCamera.CamMode;
import team3647.lib.vision.IVisionCamera.LEDMode;

public class Limelight implements AprilTagCamera {

    // NetworkTable is the class used to grab values from the Limelight Network
    // Table
    private final NetworkTable table;
    private final double[] emptyDoubleArray = {};
    private final double extraLatencySec;
    private final String ip;
    private final CamConstants kCamConstants;
    private boolean validEntry = false;
    private double skew = 0.0;
    private double tx = 0.0;
    private double ty = 0.0;

    private List<VisionPoint> corners;
    private double captureTimestamp = 0.0;
    private VisionPipeline currentPipeline = new VisionPipeline(0, 960, 720);

    public enum Data {
        VALID_TARGET("tv"),
        X("tx"),
        Y("ty"),
        AREA("ta"),
        SKEW("ts"),
        LATNECY_MS("tl"),
        RAW_CORNERS("tcornxy"),
        TAG_ID("tid"),
        CAM_TRANS("camtran");

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
        table.getEntry(Data.LATNECY_MS.str);
    }

    public synchronized void writeToInputs(VisionInputs inputs) {
        inputs.corners = this.corners;
        inputs.captureTimestamp = captureTimestamp;
        inputs.validEntry = validEntry;
        inputs.skew = this.skew;
        inputs.angleToVisionCenter = -tx;
        inputs.pitchToVisionCenter = ty;
    }

    public StampedPose getRobotPose() {
        return new StampedPose(new Pose2d(), captureTimestamp);
    }

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
}
