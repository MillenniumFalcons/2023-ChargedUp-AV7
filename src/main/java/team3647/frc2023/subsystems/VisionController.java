// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map;
import java.util.function.Consumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.AprilTagCamera;
import team3647.lib.vision.IVisionCamera.VisionPipeline;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.Limelight.Data;

/** Add your docs here. */
public class VisionController implements PeriodicSubsystem {
    public enum CAMERA_NAME {
        CENTER,
        LEFT,
        RIGHT,
        ALL
    }

    public class PeriodicIO {
        Pose2d averagedPose = new Pose2d();
    }

    public static class VisionInput {
        public Pose2d pose;
        public double timestamp;
        public CAMERA_NAME name;

        public VisionInput(double timestamp, Pose2d pose, CAMERA_NAME name) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.name = name;
        }
    }

    public VisionController(
            Map<CAMERA_NAME, Limelight> cameras, Consumer<VisionInput> sendVisionUpdate) {
        this.cameras = cameras;
        this.visionUpdate = sendVisionUpdate;
    }

    @Override
    public void readPeriodicInputs() {
        var totX = 0.0;
        var totY = 0.0;
        var totAngle = 0.0;
        var totTimestamp = 0.0;
        var divideBy = 0.0;
        for (var name : cameras.keySet()) {
            var camera = cameras.get(name);
            var stampedPose = camera.getRobotPose();
            if (stampedPose == AprilTagCamera.KNoAnswer) {
                continue;
            }
            totTimestamp += stampedPose.timestamp;
            totX += stampedPose.pose.getX();
            totY += stampedPose.pose.getY();
            totAngle += stampedPose.pose.getRotation().getRadians();
            divideBy += 1;
        }

        if (divideBy != 0) {
            periodicIO.averagedPose =
                    new Pose2d(
                            totX / divideBy,
                            totY / divideBy,
                            Rotation2d.fromRadians(totAngle / divideBy));
            var timestamp = totTimestamp / divideBy;
            VisionInput input =
                    new VisionInput(timestamp, periodicIO.averagedPose, CAMERA_NAME.ALL);
            visionUpdate.accept(input);
        }
    }

    public void changePipeline(CAMERA_NAME name, VisionPipeline pipeLine) {
        if (cameras.containsKey(name)) {
            this.cameras.get(name).setPipeline(pipeLine);
        }
    }

    public double getXToTape(CAMERA_NAME name) {
        if (!cameras.containsKey(name)) {
            return 0;
        }
        var camera = cameras.get(name);

        if (camera.getPipeline().asInt == 1) {
            return camera.getDouble(Data.X);
        }
        return 0;
    }

    public double getCurrentPipeline(CAMERA_NAME name) {
        if (!cameras.containsKey(name)) {
            return -1;
        }
        return cameras.get(name).getPipeline().asInt;
    }

    @Override
    public String getName() {
        return "VisionController";
    }

    public Pose2d getAveragedPose() {
        return periodicIO.averagedPose;
    }

    private final Map<CAMERA_NAME, Limelight> cameras;
    private final Consumer<VisionInput> visionUpdate;
    private PeriodicIO periodicIO = new PeriodicIO();
}
