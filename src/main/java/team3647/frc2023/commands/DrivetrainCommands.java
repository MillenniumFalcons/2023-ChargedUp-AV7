package team3647.frc2023.commands;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2023.subsystems.SwerveDrive;

public class DrivetrainCommands {

    public Command balance(PIDController pitchController, PIDController rollController) {

        return Commands.runEnd(
                () -> {
                    if (swerve.isBalanced(3)) {
                        return;
                    } else {

                        double pitchPID = pitchController.calculate(swerve.getPitch(), -1.71);
                        double rollPID = rollController.calculate(-swerve.getRoll(), 2.15);

                        SmartDashboard.putNumber("PITCH PID", pitchPID);
                        SmartDashboard.putNumber("ROLL PID", rollPID);
                        swerve.drive(new Translation2d(pitchPID, rollPID), 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public Command drive(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            DoubleSupplier slowTriggerFunction,
            BooleanSupplier getIsFieldOriented,
            BooleanSupplier shouldFlip) {
        return Commands.run(
                () -> {
                    double triggerSlow = 1.0 - (slowTriggerFunction.getAsDouble()) + 0.1;
                    var translation =
                            new Translation2d(
                                            ySpeedFunction.getAsDouble(),
                                            -xSpeedFunction.getAsDouble())
                                    .times(swerve.getMaxSpeedMpS())
                                    .times(triggerSlow);
                    translation =
                            shouldFlip.getAsBoolean()
                                    ? translation.rotateBy(OneEightyRotation)
                                    : translation;
                    var rotation =
                            -turnSpeedFunction.getAsDouble()
                                    * swerve.getMaxRotationRadpS()
                                    * triggerSlow;
                    swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), true);
                },
                swerve);
    }

    public Command robotRelativeDrive(Translation2d t, double seconds) {
        return Commands.run(() -> swerve.drive(t, 0, false, true), swerve)
                .finallyDo(interupted -> swerve.end())
                .withTimeout(seconds);
    }

    public Command toPointCommand(Supplier<PathPoint> selectedPoint) {
        return new ProxyCommand(
                () -> {
                    var point = selectedPoint.get();
                    return swerve.getTrajectoryCommand(swerve.getToPointATrajectory(point));
                });
    }

    private final SwerveDrive swerve;
    private static final Rotation2d OneEightyRotation = Rotation2d.fromDegrees(180);

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }
}
