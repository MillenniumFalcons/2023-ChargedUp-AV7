package team3647.lib.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Inspired by (basically copied lol) 254 RobotState class */
public class RobotTracker {
    private final double kBufferLengthSeconds;

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobot;
    private final Supplier<Pose2d> drivetrainGetPose;
    private Pose2d previousPose;
    private final DoubleSupplier drivetrainGetPoseTS;

    private Twist2d measuredVelocity = new Twist2d();

    public RobotTracker(
            double bufferLengthSeconds,
            Supplier<Pose2d> drivetrainGetPose,
            DoubleSupplier drivetrainGetPoseTS) {
        this.kBufferLengthSeconds = bufferLengthSeconds;
        fieldToRobot = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
        this.drivetrainGetPose = drivetrainGetPose;
        this.drivetrainGetPoseTS = drivetrainGetPoseTS;
        previousPose = drivetrainGetPose.get();
    }

    public void clear() {
        this.fieldToRobot.clear();
    }

    public void update() {
        addFieldToRobotObservation(drivetrainGetPoseTS.getAsDouble(), drivetrainGetPose.get());
    }

    public synchronized void addFieldToRobotObservation(double timestamp, Pose2d observation) {
        setRobotVelocityObservation(this.previousPose.log(observation));
        this.previousPose = observation;
        fieldToRobot.addSample(timestamp, observation);
    }

    public synchronized void setRobotVelocityObservation(Twist2d measuredVelocity) {
        this.measuredVelocity = measuredVelocity;
    }

    public synchronized Pose2d getFieldToRobot(double timestamp) {
        var ftrOption = fieldToRobot.getSample(timestamp);
        if (ftrOption.isEmpty()) {
            return null;
        }
        return ftrOption.get();
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return measuredVelocity;
    }
}
