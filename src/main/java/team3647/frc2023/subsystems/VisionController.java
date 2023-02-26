// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import java.util.function.Consumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.IVisionCamera.VisionPipeline;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.Limelight.Data;

/** Add your docs here. */
public class VisionController implements PeriodicSubsystem {
    public enum CAMERA_NAME {
        CENTER,
        LEFT,
        RIGHT
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
        for (var name : cameras.keySet()) {
            var camera = cameras.get(name);
            var stampedPose = camera.getRobotPose();
            VisionInput input = new VisionInput(stampedPose.timestamp, stampedPose.pose, name);
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

    private final Map<CAMERA_NAME, Limelight> cameras;
    private final Consumer<VisionInput> visionUpdate;
}
