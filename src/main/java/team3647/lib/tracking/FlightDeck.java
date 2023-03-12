package team3647.lib.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import team3647.lib.vision.AimingParameters;
import team3647.lib.vision.AprilTagCamera.AprilTagId;
import team3647.lib.vision.MultiTargetTracker;
import team3647.lib.vision.TrackedTarget;
import team3647.lib.vision.TrackedTarget.TrackedTargetComparator;

public class FlightDeck {
    private final RobotTracker robotTracker;
    private final MultiTargetTracker targetTracker;
    private final Transform2d kRobotToCamTransform;
    public static double maxAge;
    private AprilTagId lastTargetId = AprilTagId.ID_1;

    private static final Rotation2d kZero = Rotation2d.fromDegrees(0);
    private static final Rotation2d kOneEighty = Rotation2d.fromDegrees(180);

    public FlightDeck(
            RobotTracker robotTracker,
            MultiTargetTracker targetTracker,
            Transform2d kRobotToCamFixed) {
        this.robotTracker = robotTracker;
        this.targetTracker = targetTracker;
        kRobotToCamTransform = kRobotToCamFixed;
    }

    public synchronized void addVisionObservation(VisionInput input) {
        Pose2d fieldToRobot = robotTracker.getFieldToRobot(input.timestamp);
        if (fieldToRobot == null || input.cameraToTarget == null) {
            return;
        }
        Pose2d fieldToTarget =
                fieldToRobot.transformBy(kRobotToCamTransform).transformBy(input.cameraToTarget);
        Rotation2d targetRotation = kZero;
        if (DriverStation.getAlliance() == Alliance.Red) {
            targetRotation = kOneEighty;
        }
        targetTracker.update(
                input.timestamp,
                input.id,
                new Pose2d(fieldToTarget.getTranslation(), targetRotation));
    }

    public synchronized Pose2d getInFieldCoordinatesFromCamera(Pose2d pose) {
        Pose2d fieldToRobot = robotTracker.getFieldToRobot(Timer.getFPGATimestamp());
        if (fieldToRobot == null || pose == null) {
            return new Pose2d();
        }
        Transform2d camToGoalTransform = new Transform2d(new Pose2d(), pose);
        return fieldToRobot.transformBy(kRobotToCamTransform).transformBy(camToGoalTransform);
    }

    public Pose2d getFieldToCamera() {
        var ftr = robotTracker.getFieldToRobot(Timer.getFPGATimestamp());
        if (ftr == null) {
            return null;
        }
        return ftr.transformBy(kRobotToCamTransform);
    }

    public TrackedTarget getClosestTarget(List<TrackedTarget> targets, Pose2d robotPose) {
        return Collections.min(
                targets,
                Comparator.comparing(
                                (TrackedTarget other) ->
                                        robotPose
                                                .getTranslation()
                                                .getDistance(
                                                        other.getSmoothedPosition()
                                                                .getTranslation()))
                        .thenComparing(
                                (TrackedTarget other) ->
                                        Math.abs(
                                                robotPose
                                                        .getRotation()
                                                        .minus(
                                                                other.getSmoothedPosition()
                                                                        .getRotation())
                                                        .getRadians())));
    }

    private synchronized AimingParameters getAimingParameters(AprilTagId lastTargetId) {
        List<TrackedTarget> targets = targetTracker.getTrackedTargets();
        var currentPose = robotTracker.getFieldToRobot(Timer.getFPGATimestamp());

        if (currentPose == null) {
            return AimingParameters.None;
        }

        SmartDashboard.putNumber("Number of targets", targets.size());
        if (targets.isEmpty()) {
            return AimingParameters.None;
        }
        var lastTarget = targetTracker.getById(lastTargetId);
        var closestTarget = getClosestTarget(targets, currentPose);

        if (lastTarget == null) {
            this.lastTargetId = closestTarget.id;
            return AimingParameters.fromTarget(closestTarget, currentPose);
        }

        TrackedTargetComparator comparator =
                new TrackedTargetComparator(50, 50, Timer.getFPGATimestamp(), 100, lastTargetId);

        int val = comparator.compare(lastTarget, closestTarget);

        var bestTarget = lastTarget;
        if (val > 0) {
            bestTarget = closestTarget;
        }

        this.lastTargetId = bestTarget.id;

        return AimingParameters.fromTarget(bestTarget, currentPose);
    }

    public AimingParameters getLatestParameters() {
        return getAimingParameters(lastTargetId);
    }

    public RobotTracker getTracker() {
        return robotTracker;
    }

    public static class VisionInput {
        public Transform2d cameraToTarget;
        public double timestamp;
        public AprilTagId id;

        public VisionInput(double timestamp, AprilTagId id, Transform2d cameraToTarget) {
            this.cameraToTarget = cameraToTarget;
            this.timestamp = timestamp;
            this.id = id;
        }
    }
}
