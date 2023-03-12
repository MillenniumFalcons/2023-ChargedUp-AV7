package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import team3647.lib.vision.AprilTagCamera.AprilTagId;

/** 254 AimingParameters class */
public class AimingParameters {
    public static AimingParameters None =
            new AimingParameters(AprilTagId.ID_1, new Pose2d(), new Pose2d(), 0, 0);
    public final AprilTagId id;
    private final Pose2d fieldToTarget;
    private final Pose2d fieldToRobot;
    private final Transform2d robotToTarget;
    private final double lastSeenTimestamp;
    private final double stability;

    public static AimingParameters fromTarget(TrackedTarget target, Pose2d currentPose) {
        return new AimingParameters(
                target.id,
                currentPose,
                target.getSmoothedPosition(),
                target.getLatestTimestamp(),
                target.getStability());
    }

    public AimingParameters(
            AprilTagId id,
            Pose2d fieldToRobot,
            Pose2d fieldToTarget,
            double lastSeenTimestamp,
            double stability) {
        this.id = id;
        this.fieldToTarget = fieldToTarget;

        this.fieldToRobot = fieldToRobot;
        this.robotToTarget = fieldToTarget.minus(fieldToRobot);

        this.lastSeenTimestamp = lastSeenTimestamp;
        this.stability = stability;
    }

    public double getStability() {
        return stability;
    }

    public double getLastSeenTimestamp() {
        return lastSeenTimestamp;
    }

    public Pose2d getFieldToGoal() {
        return fieldToTarget;
    }

    public Transform2d getRobotToTargetTransform() {
        return robotToTarget;
    }

    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }
}
