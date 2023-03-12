// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import java.util.function.Consumer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.tracking.FlightDeck.VisionInput;
import team3647.lib.vision.AprilTagCamera;
import team3647.lib.vision.AprilTagCamera.AprilTagId;
import team3647.lib.vision.AprilTagCamera.VisionUpdate;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.IVisionCamera.VisionPipeline;
import team3647.lib.vision.IVisionCamera.VisionPoint;

/** Add your docs here. */
public class VisionController implements PeriodicSubsystem {
    public enum CAMERA_NAME {
        CENTER,
        LEFT,
        RIGHT,
        ALL
    }

    public VisionController(
            Map<CAMERA_NAME, AprilTagCamera> cameras,
            Consumer<VisionInput> sendVisionUpdate,
            double scoreTargetHeightMeters,
            double intakeTargetHeightMeters) {
        this.cameras = cameras;
        this.visionUpdate = sendVisionUpdate;
        this.scoreTargetHeightMeters = scoreTargetHeightMeters;
        this.intakeTargetHeightMeters = intakeTargetHeightMeters;
    }

    @Override
    public void readPeriodicInputs() {
        for (var camera : cameras.values()) {
            VisionUpdate update = camera.getVisionUpdate();

            if (update == VisionUpdate.kNoUpdate) {
                continue;
            }

            double targetHeightMeters = scoreTargetHeightMeters;
            if (update.id == AprilTagId.ID_4 || update.id == AprilTagId.ID_5) {
                targetHeightMeters = intakeTargetHeightMeters;
            }

            Translation2d camToTarget =
                    solveTranslationToTarget(
                            update.point, targetHeightMeters, camera.getConstants());
            SmartDashboard.putNumber("Cam to target X", camToTarget.getX());
            SmartDashboard.putNumber("Cam to target Y", camToTarget.getY());
            visionUpdate.accept(
                    new VisionInput(
                            update.captureTimestamp,
                            update.id,
                            new Transform2d(camToTarget, rotation2d)));
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

    public static Translation2d solveTranslationToTarget(
            VisionPoint corner, double targetHeightMeters, CamConstants camConstants) {

        var actualXy = new Translation2d(corner.x, corner.y).rotateBy(camConstants.kCamRoll);
        double floorToCamMeters = camConstants.kCameraHeightMeters;
        double angleToGoal = camConstants.kHorizontalToLens.getDegrees() + actualXy.getY();
        double range =
                (targetHeightMeters - floorToCamMeters)
                        / (Math.tan(Units.degreesToRadians(angleToGoal))
                                * Math.cos(Units.degreesToRadians(actualXy.getX())));
        return new Translation2d(range, Rotation2d.fromDegrees(-actualXy.getX()));
    }

    @Override
    public String getName() {
        return "VisionController";
    }

    private final Map<CAMERA_NAME, AprilTagCamera> cameras;
    private final Consumer<VisionInput> visionUpdate;
    private final double scoreTargetHeightMeters;
    private final double intakeTargetHeightMeters;
    private final Rotation2d rotation2d = new Rotation2d();
}
