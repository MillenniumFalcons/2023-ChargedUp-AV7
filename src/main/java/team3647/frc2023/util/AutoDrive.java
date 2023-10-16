package team3647.frc2023.util;

import com.google.common.base.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.inputs.Joysticks;

public class AutoDrive {
    private Joysticks mainController;
    private Superstructure superstructure;
    private DriveMode mode = DriveMode.Manual;
    private PIDController xyController;
    private Supplier<GamePiece> gamePieceSupplier;
    private SwerveDrive swerve;
    private BooleanSupplier triggerSlow;
    private DoubleSupplier txSupplier;

    public AutoDrive(
            Joysticks mainController,
            Superstructure superstructure,
            PIDController xyController,
            Supplier<GamePiece> gamePieceSupplier,
            SwerveDrive swerve,
            BooleanSupplier triggerSlow,
            DoubleSupplier txSupplier) {
        this.mainController = mainController;
        this.superstructure = superstructure;
        this.xyController = xyController;
        this.gamePieceSupplier = gamePieceSupplier;
        this.swerve = swerve;
        this.triggerSlow = triggerSlow;
        this.txSupplier = txSupplier;
    }

    public boolean getIsManual() {
        return mode == DriveMode.Manual;
    }

    public double getSlowMultiplier() {
        return triggerSlow.getAsBoolean() ? 0.5 : 1;
    }

    public double getXVelocity() {
        switch (this.mode) {
            default:
                return 0;
            case Manual:
                return mainController.getLeftStickX() * getSlowMultiplier() * 5;
            case Intake:
                return mainController.getLeftStickX() * getSlowMultiplier() * 5;
            case Score:
                return mainController.getLeftStickX() * getSlowMultiplier() * 5;
            case Balance:
                return 0;
        }
    }

    public double getYVelocity() {
        switch (this.mode) {
            default:
                return 0;
            case Manual:
                return mainController.getLeftStickY() * getSlowMultiplier() * 5;
            case Intake:
                return mainController.getLeftStickY() * getSlowMultiplier() * 5;
            case Score:
                return (Math.abs((swerve.getHeading() % 360) - 180) < 15
                                        || Math.abs((swerve.getHeading() % 360) + 180) < 15)
                                && Math.abs(txSupplier.getAsDouble()) > 1
                        ? xyController.calculate(txSupplier.getAsDouble())
                        : 0;
            case Balance:
                return 0;
        }
    }

    public double getRotation() {
        switch (this.mode) {
            default:
                return 0;
            case Manual:
                return -mainController.getRightStickX() * getSlowMultiplier() * 10;
            case Intake:
                return Math.abs(swerve.getHeading()) < 1
                        ? 0
                        : SwerveDriveConstants.kAutoSteerHeadingController.calculate(
                                swerve.getHeading());
            case Score:
                return (Math.abs((swerve.getHeading() % 360) - 180) < 1
                                || Math.abs((swerve.getHeading() % 360) + 180) < 1)
                        ? 0
                        : SwerveDriveConstants.kAutoSteerHeadingController.calculate(
                                swerve.getHeading() - 180);
            case Balance:
                return 0;
        }
    }

    public Twist2d getVelocities() {
        return new Twist2d(getXVelocity(), getYVelocity(), getRotation());
    }

    public Command changeMode(DriveMode mode) {
        return Commands.run(() -> this.mode = mode);
    }

    public final Trigger goodForLockScore =
            mainController
                    .buttonY
                    .and(mainController.rightTrigger)
                    .and(() -> !superstructure.isBottomF())
            //     .and(() -> superstructure.getGamePiece() == GamePiece.Cone)
            //     .and(() -> superstructure.getWantedLevel() != Level.Ground);
            ;
    public final Trigger goodForLockIntake =
            mainController.rightBumper.and(
                    () ->
                            (superstructure.getWantedStation() == StationType.Double
                                    || superstructure.getWantedIntakePiece() == GamePiece.Cone));

    public enum DriveMode {
        Manual,
        Intake,
        Score,
        Balance
    }
}
