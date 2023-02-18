// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.AprilTagCamera;

/** Add your docs here. */
public class VisionController implements PeriodicSubsystem {
    public VisionController(AprilTagCamera camera, BiConsumer<Double, Pose2d> sendVisionUpdate) {
        this.camera = camera;
        this.visionUpdate = sendVisionUpdate;
    }

    @Override
    public void readPeriodicInputs() {
        var stampedPose = camera.getRobotPose();
        visionUpdate.accept(stampedPose.timestamp, stampedPose.pose);
    }

    @Override
    public String getName() {
        return "VisionController";
    }

    private final AprilTagCamera camera;
    private final BiConsumer<Double, Pose2d> visionUpdate;
}
