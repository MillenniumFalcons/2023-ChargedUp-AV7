package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

                    if(Math.abs(swerve.getRawHeading()) > 5) {

                    

                    var translation =
                            new Translation2d(
                                    Math.cos(Units.degreesToRadians(swerve.getRawHeading())) * rollController.calculate(-swerve.getRoll(), 0),
                                    Math.sin(Units.degreesToRadians(swerve.getRawHeading())) * rollController.calculate(swerve.getRoll(), 0));// + pitchController.calculate(-swerve.getPitch(), 0));
                    swerve.drive(translation, 0, false, false);

                    }

                    else {
                        var translation =
                        new Translation2d(
                                rollController.calculate(-swerve.getRoll(), 0),
                                0);// + pitchController.calculate(-swerve.getPitch(), 0));
                swerve.drive(translation, 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }
}
