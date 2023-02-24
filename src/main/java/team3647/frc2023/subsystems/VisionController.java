// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
            Limelight camera, Consumer<VisionInput> sendVisionUpdate, CAMERA_NAME name) {
        this.camera = camera;
        this.visionUpdate = sendVisionUpdate;
        this.name = name;
    }

    @Override
    public void readPeriodicInputs() {
        var stampedPose = camera.getRobotPose();
        VisionInput input = new VisionInput(stampedPose.timestamp, stampedPose.pose, name);
        visionUpdate.accept(input);
    }

    public void changePipeline(VisionPipeline pipeLine) {
        this.camera.setPipeline(pipeLine);
    }

    public double getXToTape() {
        if (this.camera.getPipeline().asInt == 1) {
            return this.camera.getDouble(Data.X);
        }
        return 0;
    }

    public double getCurrentPipeline() {
        return this.camera.getPipeline().asInt;
    }

    @Override
    public String getName() {
        return "VisionController";
    }

    private final Limelight camera;
    private final Consumer<VisionInput> visionUpdate;
    private final CAMERA_NAME name;
}
