package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.subsystems.SwerveDrive;

public class DrivetrainCommands {

    private final SwerveDrive swerve;

    public Command balance(PIDController pitchController, PIDController rollController) {

        return Commands.runEnd(
                () -> {
                    if (swerve.isBalanced(1)) {
                        return;
                    }

                    var translation =
                            new Translation2d(
                                    pitchController.calculate(swerve.getPitch(), 0),
                                    rollController.calculate(swerve.getRoll(), 0));
                    swerve.drive(translation, 0, false, false);
                },
                swerve::stopModules,
                swerve);
    }

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }
}
