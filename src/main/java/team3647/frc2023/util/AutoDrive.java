package team3647.frc2023.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoDrive {
    private PIDController xyController;
    private SwerveDrive swerve;
    private DoubleSupplier txSupplier;
    private boolean enabled = true;
    private boolean lockY;
    private boolean lockRotation;
    private double turnSetpoint;

    public AutoDrive(PIDController xyController, SwerveDrive swerve, DoubleSupplier txSupplier) {
        this.xyController = xyController;
        this.swerve = swerve;
        this.txSupplier = txSupplier;
        xyController.setTolerance(1);
    }

    public boolean isAlmostDone() {
        return (Math.abs((swerve.getHeading() % 360) - 180) < 15
                        || Math.abs((swerve.getHeading() % 360) + 180) < 15)
                && Math.abs(txSupplier.getAsDouble()) > 1;
    }

    public boolean getLockY() {
        return lockY && enabled;
    }

    public boolean getLockRot() {
        return lockRotation && enabled;
    }

    public boolean isAllDisabled() {
        return !lockY && !lockRotation;
    }

    public Command lockY() {
        return Commands.runOnce(() -> this.lockY = true);
    }

    public boolean doneRotatingGruond() {
        return Math.abs((swerve.getHeading() % 360) - 90) < 1
                || Math.abs((swerve.getHeading() % 360) + 270) < 1;
    }

    public Command unlockY() {
        return Commands.runOnce(() -> this.lockY = false);
    }

    public Command lockRot(double setpoint) {
        return Commands.sequence(
                Commands.runOnce(() -> this.lockRotation = true),
                Commands.run(
                                () -> {
                                    turnSetpoint =
                                            (Math.abs((swerve.getHeading() % 360) - setpoint) < 1
                                                            || Math.abs(
                                                                            (swerve.getHeading()
                                                                                            % 360)
                                                                                    + setpoint)
                                                                    < 1)
                                                    ? 0
                                                    : SwerveDriveConstants
                                                            .kAutoSteerHeadingController
                                                            .calculate(
                                                                    swerve.getHeading() - setpoint);
                                })
                        .until(() -> this.lockRotation == false));
    }

    public Command unlockRot() {
        return Commands.runOnce(() -> this.lockRotation = false);
    }

    public Command enable() {
        return Commands.runOnce(() -> this.enabled = true);
    }

    public Command disable() {
        return Commands.runOnce(() -> this.enabled = false);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getYVelocity() {
        return lockY && enabled && Math.abs(txSupplier.getAsDouble()) > 1
                ? xyController.calculate(txSupplier.getAsDouble())
                : 0;
    }

    public double getRotation() {
        return lockRotation && enabled ? turnSetpoint : 0;
    }

    public Twist2d getVelocities() {
        return new Twist2d(0, getYVelocity(), getRotation());
    }
}
