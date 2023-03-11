// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import java.util.function.Consumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.tracking.FlightDeck.VisionInput;
import team3647.lib.vision.AprilTagCamera;
import team3647.lib.vision.AprilTagCamera.AprilTagId;
import team3647.lib.vision.IVisionCamera.VisionPipeline;

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

    public VisionController(
            Map<CAMERA_NAME, AprilTagCamera> cameras, Consumer<VisionInput> sendVisionUpdate) {
        this.cameras = cameras;
        this.visionUpdate = sendVisionUpdate;
    }

    @Override
    public void readPeriodicInputs() {
        for (AprilTagCamera camera : cameras.values()) {
            var tags = camera.getCamToTags();
            if (tags == AprilTagCamera.kNoTags) {
                return;
            }
            for (var entry : tags.entrySet()) {
                var id = entry.getKey();
                if (id == AprilTagId.ID_DNE) {
                    return;
                }
                var camToTagTransform = entry.getValue();

                this.visionUpdate.accept(
                        new VisionInput(
                                camToTagTransform.timestamp, id, camToTagTransform.transform));
            }
        }
    }

    public void changePipeline(CAMERA_NAME name, VisionPipeline pipeLine) {
        if (cameras.containsKey(name)) {
            this.cameras.get(name).setPipeline(pipeLine);
        }
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

    private final Map<CAMERA_NAME, AprilTagCamera> cameras;
    private final Consumer<VisionInput> visionUpdate;
    private PeriodicIO periodicIO = new PeriodicIO();
}
