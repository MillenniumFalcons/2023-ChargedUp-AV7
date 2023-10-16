package team3647.frc2023.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.robot.PositionFinder;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.subsystems.CubeShooterTop;
import team3647.frc2023.subsystems.Rollers;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.inputs.Joysticks;
import team3647.lib.vision.LimelightHelpers;

public class LimelightTriggers {
    private Superstructure superstructure;
    private Rollers rollers;
    private CubeShooterTop cubeShooterTop;
    private Joysticks mainController;
    private Joysticks coController;
    private SwerveDrive swerve;

    public LimelightTriggers(
            Superstructure superstructure,
            Rollers rollers,
            CubeShooterTop cubeShooterTop,
            Joysticks mainController,
            Joysticks coController,
            SwerveDrive swerve) {
        this.superstructure = superstructure;
        this.rollers = rollers;
        this.cubeShooterTop = cubeShooterTop;
        this.mainController = mainController;
        this.coController = coController;
        this.swerve = swerve;
    }

    public final Trigger currentCone =
            new Trigger(
                    () ->
                            (superstructure.getWantedIntakePiece() == GamePiece.Cone
                                    && rollers.getMasterCurrent() > 50));
    public final Trigger currentCube =
            new Trigger(
                    () ->
                            (superstructure.getWantedIntakePiece() == GamePiece.Cube
                                    && rollers.getMasterCurrent() > 50));

    public final Trigger currentGroundCube =
            new Trigger(() -> (cubeShooterTop.getMasterCurrent() > 11.5));
    public final Trigger wantedCone =
            new Trigger(
                    () -> superstructure.getWantedIntakePiece() == PositionFinder.GamePiece.Cone);
    public final Trigger wantedCube =
            new Trigger(
                    () -> superstructure.getWantedIntakePiece() == PositionFinder.GamePiece.Cube);

    public final Trigger wantedGroundCube = mainController.leftBumper;

    public final Trigger isAligned =
            new Trigger(
                    () ->
                            (Math.abs(
                                                    -LimelightHelpers.getTX(
                                                            LimelightConstant.kLimelightCenterHost))
                                            < 2)
                                    && (Math.abs(
                                                    -LimelightHelpers.getTX(
                                                            LimelightConstant.kLimelightCenterHost))
                                            != 0)
                                    && (Math.abs((swerve.getHeading() % 360) - 180) < 5
                                            || Math.abs((swerve.getHeading() % 360) + 180) < 5));

    public final Trigger seesTarget =
            new Trigger(
                    () ->
                            (LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost) != 0
                                    && !isAligned.getAsBoolean()));

    public final Trigger blind =
            new Trigger(() -> !(isAligned.getAsBoolean() || seesTarget.getAsBoolean()));
    public final Trigger coControllerRightJoystickMoved =
            new Trigger(() -> Math.abs(coController.getRightStickY()) >= 0.2);
}
