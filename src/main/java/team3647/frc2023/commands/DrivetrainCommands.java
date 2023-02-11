package team3647.frc2023.commands;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.SwerveDrive;

public class DrivetrainCommands {

    public Command balance(PIDController pitchController, PIDController rollController) {

        return Commands.runEnd(
                () -> {
                    if (swerve.isBalanced(1)) {
                        return;
                    }

                    if (Math.abs(swerve.getRawHeading()) > 5) {

                        var translation =
                                new Translation2d(
                                        Math.cos(Units.degreesToRadians(swerve.getRawHeading()))
                                                * rollController.calculate(-swerve.getRoll(), 0),
                                        Math.sin(Units.degreesToRadians(swerve.getRawHeading()))
                                                * rollController.calculate(
                                                        swerve.getRoll(), 0)); // +
                        // pitchController.calculate(-swerve.getPitch(), 0));
                        swerve.drive(translation, 0, false, false);
                    } else {
                        var translation =
                                new Translation2d(
                                        rollController.calculate(-swerve.getRoll(), 0), 0);
                        swerve.drive(translation, 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public Command drive(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            BooleanSupplier getIsFieldOriented,
            BooleanSupplier shouldFlip) {
        return Commands.run(
                () -> {
                    var translation =
                            new Translation2d(
                                            ySpeedFunction.getAsDouble(),
                                            -xSpeedFunction.getAsDouble())
                                    .times(swerve.getMaxSpeedMpS());
                    translation =
                            shouldFlip.getAsBoolean()
                                    ? translation.rotateBy(
                                            new Rotation2d(Units.degreesToRadians(180)))
                                    : translation;
                    var rotation = -turnSpeedFunction.getAsDouble() * swerve.getMaxRotationRadpS();
                    swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), true);
                },
                swerve);
    }

    public Command robotRelativeDrive(Translation2d t, double seconds) {
        return Commands.run(() -> swerve.drive(t, 0, false, true), swerve)
                .finallyDo(interupted -> swerve.end())
                .withTimeout(seconds);
    }

    public Command getToPointCommand(PathPoint point) {
        return new InstantCommand(
                () -> {
                    new PrintCommand("Starting!")
                            .andThen(
                                    swerve.getTrajectoryCommand(swerve.getToPointATrajectory(point))
                                            .withTimeout(8))
                            .schedule();
                });
    }

    private final SwerveDrive swerve;

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }
}
