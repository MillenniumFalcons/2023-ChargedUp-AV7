package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

                        // SmartDashboard.putNumber("PITCH PID", pitchPID);
                        // SmartDashboard.putNumber("ROLL PID", rollPID);
                        swerve.drive(new Translation2d(rollPID, pitchPID), 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public Command driveVisionTeleop(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            BooleanSupplier slowTriggerFunction,
            BooleanSupplier enableAutoSteer,
            BooleanSupplier getIsFieldOriented,
            BooleanSupplier shouldFlip,
            Supplier<Twist2d> autoSteerVelocitiesSupplier) {
        return Commands.run(
                () -> {
                    double triggerSlow = slowTriggerFunction.getAsBoolean() ? 0.2 : 1;
                    boolean autoSteer = enableAutoSteer.getAsBoolean();
                    boolean fieldOriented = getIsFieldOriented.getAsBoolean();
                    var motionXComponent = ySpeedFunction.getAsDouble() * maxSpeed * triggerSlow;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -xSpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    var motionTurnComponent =
                            -turnSpeedFunction.getAsDouble() * maxRotationRadpS * triggerSlow;

                    var translation = new Translation2d(motionXComponent, motionYComponent);
                    translation =
                            shouldFlip.getAsBoolean() && fieldOriented
                                    ? translation.rotateBy(OneEightyRotation)
                                    : translation;

                    if (autoSteer && fieldOriented) {
                        var autoSteerVelocities = autoSteerVelocitiesSupplier.get();
                        // completely take over rotation for heading lock unless driver wants to
                        // change setpoint
                        motionTurnComponent =
                                Math.abs(motionTurnComponent) < .1
                                        ? autoSteerVelocities.dtheta
                                                + Math.signum(autoSteerVelocities.dtheta) * 0.1
                                        : motionTurnComponent;

                        if (Math.abs(motionXComponent) > 0.1 || Math.abs(motionYComponent) > 0.1) {
                            motionXComponent =
                                    motionXComponent * 0.5
                                            + autoSteerVelocities.dx
                                            + Math.signum(autoSteerVelocities.dx) * 0.25;
                            motionYComponent =
                                    motionYComponent * 0.5
                                            + autoSteerVelocities.dy
                                            + Math.signum(autoSteerVelocities.dy) * 0.25;
                            SmartDashboard.putNumber(
                                    "autoSteerVelocities.dx", autoSteerVelocities.dx);
                            SmartDashboard.putNumber(
                                    "autoSteerVelocities.dy", autoSteerVelocities.dy);
                        }
                    }
                    SmartDashboard.putNumber("wanted x", translation.getY());
                    var rotation = motionTurnComponent;
                    swerve.drive(translation, rotation, fieldOriented, true);
                },
                swerve);
    }

    public Command robotRelativeDrive(Translation2d t, double seconds) {
        return Commands.run(() -> swerve.drive(t, 0, false, true), swerve)
                .finallyDo(interupted -> swerve.end())
                .withTimeout(seconds);
    }

    public Command rotateToTape(PIDController yController, DoubleSupplier angle, double offset) {
        return Commands.run(
                () -> {
                    double distance = Math.sin(Units.degreesToRadians(angle.getAsDouble()));
                    double yPID = yController.calculate(distance, offset);
                    swerve.drive(new Translation2d(0, yPID), 0, false, false);
                },
                swerve);
    }

    private final SwerveDrive swerve;
    private static final Rotation2d OneEightyRotation = Rotation2d.fromDegrees(180);
    private final double maxSpeed;
    private final double maxRotationRadpS;

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
        this.maxSpeed = this.swerve.getMaxSpeedMpS();
        this.maxRotationRadpS = this.swerve.getMaxRotationRadpS();
    }
}
